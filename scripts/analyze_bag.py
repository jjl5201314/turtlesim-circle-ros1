#!/usr/bin/env python3
"""Plot ROS1 turtlesim recordings without resampling or requiring a ROS install.

Twist and turtlesim/Pose do not contain Header timestamps. All time axes use
the recorder's bag receive timestamps, relative to the earliest selected sample.
"""

import argparse
import csv
import hashlib
import importlib
import json
import math
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path
import numpy as np


CMD_FIELDS = ("linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z")
POSE_FIELDS = ("x", "y", "theta", "linear_velocity", "angular_velocity")
CMD_TYPE = "geometry_msgs/Twist"
POSE_TYPE = "turtlesim/Pose"
PROVENANCE_TOPIC = "/turtlesim_circle/synthetic_provenance"
ARTIFACTS = ("velocity_vs_time.png", "position_vs_time.png", "trajectory_xy.png", "cmd_vel.csv", "pose.csv", "summary.json")


class AnalysisError(Exception):
    """A recording cannot be analyzed as the requested turtlesim experiment."""


@dataclass
class Samples:
    timestamp_ns: np.ndarray
    values: np.ndarray


@dataclass
class Recording:
    command: Samples
    pose: Samples
    origin_ns: int
    cmd_topic: str
    pose_topic: str
    backend: str
    provenance: dict = None


def canonical_type(message_type):
    """rosbags normalizes ROS1 type names to the ROS2 '/msg/' form."""
    return message_type.replace("/msg/", "/")


def choose_backend(requested):
    if requested not in ("auto", "rosbag", "rosbags"):
        raise AnalysisError("Unknown backend: {}".format(requested))
    choices = ("rosbag", "rosbags") if requested == "auto" else (requested,)
    failures = []
    for name in choices:
        module_name = "rosbag" if name == "rosbag" else "rosbags.highlevel"
        try:
            return name, importlib.import_module(module_name)
        except ImportError as exc:
            failures.append("{}: {}".format(name, exc))
    raise AnalysisError(
        "No usable bag reader. In ROS1, source your ROS environment; outside ROS, "
        "install the analysis requirements (rosbags, numpy, matplotlib). "
        + "; ".join(failures)
    )


def validate_topics(topic_types, cmd_topic, pose_topic):
    for topic, expected_type in ((cmd_topic, CMD_TYPE), (pose_topic, POSE_TYPE)):
        if topic not in topic_types:
            raise AnalysisError(
                "Missing topic '{}'. Available topics: {}".format(
                    topic, ", ".join(sorted(topic_types)) or "(none)"
                )
            )
        found = {canonical_type(value) for value in topic_types[topic]}
        if found != {expected_type}:
            raise AnalysisError(
                "Topic '{}' must contain {}, found {}".format(
                    topic, expected_type, ", ".join(sorted(found))
                )
            )
    if PROVENANCE_TOPIC in topic_types:
        if {canonical_type(value) for value in topic_types[PROVENANCE_TOPIC]} != {"std_msgs/String"}:
            raise AnalysisError("Synthetic provenance topic must contain std_msgs/String")


def iter_rosbag(path, module, cmd_topic, pose_topic):
    with module.Bag(str(path), "r") as bag:
        information = bag.get_type_and_topic_info().topics
        validate_topics(
            {topic: {info.msg_type} for topic, info in information.items()},
            cmd_topic,
            pose_topic,
        )
        for topic, message, timestamp in bag.read_messages(topics=[cmd_topic, pose_topic, PROVENANCE_TOPIC]):
            yield topic, int(timestamp.to_nsec()), message


def iter_rosbags(path, module, cmd_topic, pose_topic):
    # AnyReader registers turtlesim/Pose using the definition embedded in the bag.
    with module.AnyReader([path]) as reader:
        topic_types = {}
        for connection in reader.connections:
            topic_types.setdefault(connection.topic, set()).add(connection.msgtype)
        validate_topics(topic_types, cmd_topic, pose_topic)
        selected = [
            connection for connection in reader.connections
            if connection.topic in (cmd_topic, pose_topic, PROVENANCE_TOPIC)
        ]
        for connection, timestamp, raw_data in reader.messages(connections=selected):
            yield connection.topic, int(timestamp), reader.deserialize(raw_data, connection.msgtype)


def collect_samples(records, cmd_topic, pose_topic, backend="in_memory"):
    """Collect and stably order samples without aligning the two topic streams."""
    commands = []
    poses = []
    provenance = None
    for topic, timestamp_ns, message in records:
        if topic == PROVENANCE_TOPIC:
            declared = json.loads(message.data)
            if not isinstance(declared, dict) or declared.get("synthetic") is not True:
                raise AnalysisError("Invalid synthetic provenance message")
            if provenance is not None and provenance != declared:
                raise AnalysisError("Conflicting synthetic provenance messages")
            provenance = declared
            continue
        if topic not in (cmd_topic, pose_topic):
            continue
        if timestamp_ns < 0:
            raise AnalysisError("Negative bag timestamp on topic '{}'".format(topic))
        if topic == cmd_topic:
            values = (
                message.linear.x, message.linear.y, message.linear.z,
                message.angular.x, message.angular.y, message.angular.z,
            )
            destination = commands
        else:
            values = tuple(getattr(message, field) for field in POSE_FIELDS)
            destination = poses
        if not all(math.isfinite(float(value)) for value in values):
            raise AnalysisError("Non-finite message value on topic '{}'".format(topic))
        destination.append((int(timestamp_ns), values))

    streams = []
    for topic, rows in ((cmd_topic, commands), (pose_topic, poses)):
        if not rows:
            raise AnalysisError("Topic '{}' contains no samples".format(topic))
        rows.sort(key=lambda row: row[0])
        streams.append(Samples(
            timestamp_ns=np.asarray([row[0] for row in rows], dtype=np.int64),
            values=np.asarray([row[1] for row in rows], dtype=float),
        ))
    origin_ns = min(int(stream.timestamp_ns[0]) for stream in streams)
    return Recording(streams[0], streams[1], origin_ns, cmd_topic, pose_topic, backend, provenance)


def read_recording(path, backend="auto", cmd_topic="/turtle1/cmd_vel", pose_topic="/turtle1/pose"):
    path = Path(path).expanduser().resolve()
    if cmd_topic == pose_topic:
        raise AnalysisError("Command and pose topics must be different")
    if PROVENANCE_TOPIC in (cmd_topic, pose_topic):
        raise AnalysisError("Command and pose topics cannot use the reserved synthetic provenance topic")
    if not path.is_file():
        raise AnalysisError("Bag file does not exist: {}".format(path))
    if path.suffix.lower() != ".bag":
        raise AnalysisError("Select a finalized ROS1 .bag file (stop rosbag record first)")
    with path.open("rb") as handle:
        if handle.read(13) != b"#ROSBAG V2.0\n":
            raise AnalysisError("Input is not a ROS1 bag version 2.0 file: {}".format(path))
    selected_backend, module = choose_backend(backend)
    reader = iter_rosbag if selected_backend == "rosbag" else iter_rosbags
    try:
        return collect_samples(reader(path, module, cmd_topic, pose_topic), cmd_topic, pose_topic, selected_backend)
    except AnalysisError:
        raise
    except Exception as exc:
        raise AnalysisError("Cannot read '{}' using {}: {}. Ensure recording was stopped cleanly and the bag is complete.".format(
            path.name, selected_backend, exc
        )) from exc


def relative_seconds(samples, origin_ns):
    # Subtract integer nanoseconds first, preserving small offsets at epoch time.
    return (samples.timestamp_ns - origin_ns).astype(np.float64) / 1e9


def write_csv(path, samples, fields, origin_ns, extra_columns=None):
    extra_columns = extra_columns or []
    times = relative_seconds(samples, origin_ns)
    with Path(path).open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["bag_time_ns", "time_s"] + list(fields) + [name for name, _ in extra_columns])
        for index, values in enumerate(samples.values):
            writer.writerow(
                [int(samples.timestamp_ns[index]), float(times[index])]
                + [float(value) for value in values]
                + [float(column[index]) for _, column in extra_columns]
            )


def describe(values):
    return {"min": float(np.min(values)), "max": float(np.max(values)), "mean": float(np.mean(values))}


def describe_stream(samples, origin_ns):
    times = relative_seconds(samples, origin_ns)
    intervals = np.diff(times)
    return {
        "samples": len(samples.timestamp_ns),
        "first_time_s": float(times[0]),
        "last_time_s": float(times[-1]),
        "duration_s": float(times[-1] - times[0]),
        "duplicate_timestamp_count": int(np.count_nonzero(intervals == 0)),
        "max_sample_gap_s": float(np.max(intervals)) if len(intervals) else None,
    }


def file_sha256(path):
    checksum = hashlib.sha256()
    with Path(path).open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            checksum.update(chunk)
    return checksum.hexdigest()


def build_summary(recording, bag_path):
    bag_path = Path(bag_path).expanduser().resolve()
    command, pose = recording.command, recording.pose
    unwrapped = np.unwrap(pose.values[:, 2])
    duration = (max(int(command.timestamp_ns[-1]), int(pose.timestamp_ns[-1])) - recording.origin_ns) / 1e9
    provenance_path = Path(bag_path).resolve().with_suffix(".provenance.json")
    sidecar = None
    if provenance_path.is_file():
        sidecar = json.loads(provenance_path.read_text(encoding="utf-8"))
        if not isinstance(sidecar, dict):
            raise AnalysisError("Provenance sidecar must be a JSON object: {}".format(provenance_path))
    checksum = file_sha256(bag_path) if Path(bag_path).is_file() else None
    if sidecar and sidecar.get("bag_sha256") and sidecar["bag_sha256"] != checksum:
        raise AnalysisError("Provenance sidecar SHA-256 does not match the bag; select the matching files")
    provenance = recording.provenance if recording.provenance is not None else sidecar
    if recording.provenance and sidecar and sidecar.get("synthetic") is not True:
        raise AnalysisError("Embedded synthetic provenance conflicts with the sidecar")
    synthetic = provenance is not None and provenance.get("synthetic") is True
    summary = {
        "bag_file": Path(bag_path).name,
        "bag_sha256": checksum,
        "data_origin": "SYNTHETIC DEMO: ideal kinematics, not a live ROS recording" if synthetic else "Input origin not independently verified by the analyzer",
        "synthetic": True if synthetic else None,
        "provenance_file": provenance_path.name if sidecar is not None else None,
        "provenance_source": "embedded bag declaration" if recording.provenance else ("sidecar declaration" if sidecar else "unspecified"),
        "reader_backend": recording.backend,
        "time_source": "bag receive timestamps; neither message type has a Header",
        "time_origin": "earliest recorded sample of the two selected topics",
        "origin_bag_time_ns": recording.origin_ns,
        "selected_duration_s": duration,
        "processing": "streams retain their own sample times; no numerical interpolation or resampling; plot lines only connect samples",
        "mean_definition": "unweighted sample mean for each stream, not a time-weighted mean",
        "position_unit": "turtlesim coordinate unit",
        "angle_unit": "radian",
        "command": {
            "topic": recording.cmd_topic,
            **describe_stream(command, recording.origin_ns),
            "linear_x": describe(command.values[:, 0]),
            "angular_z": describe(command.values[:, 5]),
        },
        "pose": {
            "topic": recording.pose_topic,
            **describe_stream(pose, recording.origin_ns),
            "linear_velocity": describe(pose.values[:, 3]),
            "angular_velocity": describe(pose.values[:, 4]),
            "initial_xy": pose.values[0, :2].tolist(),
            "final_xy": pose.values[-1, :2].tolist(),
            "sampled_path_length": float(np.linalg.norm(np.diff(pose.values[:, :2], axis=0), axis=1).sum()),
            "start_to_end_distance": float(np.linalg.norm(pose.values[-1, :2] - pose.values[0, :2])),
            "unwrapped_heading_change_rad": float(unwrapped[-1] - unwrapped[0]),
        },
        "limitations": [
            "Bag receive time is not exact publish or simulation update time; streams are asynchronous.",
            "Unwrapped theta assumes the heading changes by less than pi between successive pose samples.",
            "Sampled path length sums straight chords and can underestimate actual traveled distance.",
            "This analysis alone does not prove that the input was recorded from a live turtlesim session.",
        ],
        "artifacts": list(ARTIFACTS),
    }
    return summary


def write_figures(recording, output_dir, synthetic=False):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    command = recording.command.values
    pose = recording.pose.values
    command_time = relative_seconds(recording.command, recording.origin_ns)
    pose_time = relative_seconds(recording.pose, recording.origin_ns)
    x_label = "Time since first selected bag sample [s] (bag receive time)"
    title_prefix = "SYNTHETIC DEMO - no live ROS run\n" if synthetic else ""

    with plt.rc_context({"figure.dpi": 120, "savefig.dpi": 160, "axes.grid": True, "grid.alpha": 0.25}):
        fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
        for axis, command_index, pose_index, ylabel in (
            (axes[0], 0, 3, "Linear speed [units/s]"),
            (axes[1], 5, 4, "Angular speed [rad/s]"),
        ):
            axis.step(command_time, command[:, command_index], where="post", marker=".", markersize=2, label="cmd_vel (step display)", color="#1666a4")
            axis.plot(pose_time, pose[:, pose_index], ".-", markersize=2, linewidth=1, label="pose reported velocity", color="#d65b30", alpha=0.85)
            axis.set_ylabel(ylabel)
            axis.legend(loc="best")
        axes[-1].set_xlabel(x_label)
        fig.suptitle(title_prefix + "Commanded and reported velocity (original sample times)")
        fig.tight_layout()
        # Older Matplotlib in ROS Noetic does not reserve suptitle space.
        fig.subplots_adjust(top=0.88 if synthetic else 0.92)
        fig.savefig(output_dir / "velocity_vs_time.png")
        plt.close(fig)

        fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
        for axis, index, ylabel in ((axes[0], 0, "x [units]"), (axes[1], 1, "y [units]")):
            axis.plot(pose_time, pose[:, index], color="#1666a4")
            axis.set_ylabel(ylabel)
        axes[2].plot(pose_time, pose[:, 2], ".", markersize=2, label="theta (wrapped)", color="#d65b30")
        axes[2].plot(pose_time, np.unwrap(pose[:, 2]), label="theta (unwrapped)", color="#1666a4")
        axes[2].set_ylabel("Heading [rad]")
        axes[2].legend(loc="best")
        axes[-1].set_xlabel(x_label)
        fig.suptitle(title_prefix + "Turtlesim position and heading")
        fig.tight_layout()
        fig.subplots_adjust(top=0.90 if synthetic else 0.94)
        fig.savefig(output_dir / "position_vs_time.png")
        plt.close(fig)

        fig, axis = plt.subplots(figsize=(7, 7))
        axis.plot(pose[:, 0], pose[:, 1], linewidth=1, color="#1666a4", alpha=0.5)
        scatter = axis.scatter(pose[:, 0], pose[:, 1], c=pose_time, s=7, cmap="viridis")
        axis.scatter(pose[0, 0], pose[0, 1], c="#20a05b", marker="o", s=70, edgecolor="black", label="Start", zorder=5)
        axis.scatter(pose[-1, 0], pose[-1, 1], c="#d65b30", marker="X", s=90, edgecolor="black", label="End", zorder=6)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("x [turtlesim units]")
        axis.set_ylabel("y [turtlesim units]")
        axis.set_title(title_prefix + "XY trajectory from bag samples")
        axis.legend(loc="best")
        fig.colorbar(scatter, ax=axis, label="Relative bag receive time [s]", shrink=0.75)
        fig.tight_layout()
        fig.savefig(output_dir / "trajectory_xy.png")
        plt.close(fig)


def export_analysis(recording, output_dir, bag_path, overwrite=False):
    output_dir = Path(output_dir).expanduser().resolve()
    conflicts = [name for name in ARTIFACTS if (output_dir / name).exists() or (output_dir / name).is_symlink()]
    if conflicts and not overwrite:
        raise AnalysisError("Output artifacts already exist: {}. Choose a new directory or pass --overwrite.".format(
            ", ".join(conflicts)
        ))
    if any((output_dir / name).is_dir() for name in ARTIFACTS):
        raise AnalysisError("An output artifact path is a directory")
    summary = build_summary(recording, bag_path)
    output_dir.mkdir(parents=True, exist_ok=True)
    # Build the complete report first. A plotting/dependency failure must not
    # replace old CSVs while leaving stale charts and a stale summary behind.
    with tempfile.TemporaryDirectory(prefix=".analysis-", dir=str(output_dir)) as staging:
        staging = Path(staging)
        write_csv(staging / "cmd_vel.csv", recording.command, CMD_FIELDS, recording.origin_ns)
        write_csv(
            staging / "pose.csv", recording.pose, POSE_FIELDS, recording.origin_ns,
            [("theta_unwrapped", np.unwrap(recording.pose.values[:, 2]))],
        )
        write_figures(recording, staging, synthetic=summary["synthetic"] is True)
        (staging / "summary.json").write_text(
            json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
        for filename in ARTIFACTS:
            (staging / filename).replace(output_dir / filename)
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="Finalized ROS1 .bag file")
    parser.add_argument("--output-dir", required=True, type=Path, help="Directory for figures, CSVs and JSON")
    parser.add_argument("--backend", choices=("auto", "rosbag", "rosbags"), default="auto")
    parser.add_argument("--cmd-topic", default="/turtle1/cmd_vel")
    parser.add_argument("--pose-topic", default="/turtle1/pose")
    parser.add_argument("--overwrite", action="store_true", help="Replace this analyzer's six existing output files")
    args = parser.parse_args(argv)
    try:
        recording = read_recording(args.bag, args.backend, args.cmd_topic, args.pose_topic)
        summary = export_analysis(recording, args.output_dir, args.bag, overwrite=args.overwrite)
    except (AnalysisError, OSError, ImportError, ValueError, AttributeError) as exc:
        print("Analysis failed: {}".format(exc), file=sys.stderr)
        return 1
    except Exception as exc:
        # Backend libraries expose different exception classes for corrupt bags.
        print("Analysis failed ({}): {}".format(type(exc).__name__, exc), file=sys.stderr)
        return 1
    print("Analyzed {} command and {} pose samples using {}.".format(
        summary["command"]["samples"], summary["pose"]["samples"], summary["reader_backend"]
    ))
    print("Saved three PNG figures, two CSV tables and summary.json to {}".format(args.output_dir.expanduser().resolve()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
