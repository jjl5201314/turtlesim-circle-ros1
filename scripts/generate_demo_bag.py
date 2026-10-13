#!/usr/bin/env python3
"""Create a SYNTHETIC ROS1 bag to demonstrate the analysis pipeline.

This is exact continuous unicycle kinematics, not turtlesim output or a result
of running rosbag record. Recorder delays, scheduling and wall collisions are
not modeled. A provenance sidecar accompanies every generated file.
"""

import argparse
import hashlib
import json
import math
import sys
from pathlib import Path


CMD_TOPIC = "/turtle1/cmd_vel"
POSE_TOPIC = "/turtle1/pose"
PROVENANCE_TOPIC = "/turtlesim_circle/synthetic_provenance"
POSE_TYPE = "turtlesim/msg/Pose"
POSE_DEFINITION = "float32 x\nfloat32 y\nfloat32 theta\nfloat32 linear_velocity\nfloat32 angular_velocity\n"
# Fixed deterministic epoch; this is not a real experiment start time.
ORIGIN_NS = 1_700_000_000_000_000_000


def ideal_state(time_s, linear_speed, angular_speed, motion_duration, prelude=2.0):
    elapsed = min(max(time_s - prelude, 0.0), motion_duration)
    theta = angular_speed * elapsed
    radius = linear_speed / angular_speed
    x = 5.544445 + radius * math.sin(theta)
    y = 5.544445 + radius * (1.0 - math.cos(theta))
    wrapped_theta = math.atan2(math.sin(theta), math.cos(theta))
    moving = prelude <= time_s < prelude + motion_duration
    return x, y, wrapped_theta, linear_speed if moving else 0.0, angular_speed if moving else 0.0


def generate(output, amplitude=1.0, period=8.0, amplitude_mode="speed"):
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore

    if isinstance(amplitude, bool) or not math.isfinite(amplitude) or amplitude < 0:
        raise ValueError("amplitude must be finite and nonnegative")
    if isinstance(period, bool) or not math.isfinite(period) or period <= 0:
        raise ValueError("period must be finite and greater than zero")
    if amplitude_mode not in ("speed", "radius"):
        raise ValueError("amplitude_mode must be speed or radius")
    output = Path(output).expanduser().resolve()
    if output.suffix.lower() != ".bag":
        raise ValueError("Output must end with .bag")
    provenance = output.with_suffix(".provenance.json")
    if output.exists() or provenance.exists():
        raise FileExistsError("Refusing to overwrite an existing bag or provenance file: {}".format(output))
    angular_speed = 2 * math.pi / period
    linear_speed = amplitude if amplitude_mode == "speed" else angular_speed * amplitude
    radius = linear_speed / angular_speed
    motion_duration = 3 * period
    total_duration = 2.0 + motion_duration + 1.0
    if not all(math.isfinite(value) for value in (angular_speed, linear_speed, radius, total_duration)):
        raise ValueError("Parameters produce non-finite speed, radius or duration")
    if total_duration > 3600:
        raise ValueError("Synthetic demonstrations are limited to 3600 seconds")
    if max(angular_speed, linear_speed, 5.544445 + 2 * radius) > 3.4028234663852886e38:
        raise ValueError("Parameters exceed the finite range of turtlesim Pose fields")
    output.parent.mkdir(parents=True, exist_ok=True)
    typestore = get_typestore(Stores.ROS1_NOETIC)
    typestore.register(get_types_from_msg(POSE_DEFINITION, POSE_TYPE))
    vector = typestore.types["geometry_msgs/msg/Vector3"]
    twist = typestore.types["geometry_msgs/msg/Twist"]
    turtle_pose = typestore.types[POSE_TYPE]
    string_message = typestore.types["std_msgs/msg/String"]

    # Independent rates and an 8 ms pose offset make the absence of interpolation
    # in the analysis visible. Message values are sampled from the exact solution.
    events = []
    for topic, rate_hz, offset_s in ((CMD_TOPIC, 20, 0.0), (POSE_TOPIC, 60, 0.008)):
        for index in range(int(math.floor((total_duration - offset_s) * rate_hz)) + 1):
            timestamp_ns = ORIGIN_NS + round((index / rate_hz + offset_s) * 1e9)
            events.append((timestamp_ns, topic))
    # Include the exact stop command when a custom period does not land on the
    # regular 20 Hz grid, keeping the commands consistent with the ideal motion.
    stop_timestamp_ns = ORIGIN_NS + round((2.0 + motion_duration) * 1e9)
    events.append((stop_timestamp_ns, CMD_TOPIC))
    events = sorted(set(events))
    counts = {topic: sum(event_topic == topic for _, event_topic in events) for topic in (CMD_TOPIC, POSE_TOPIC)}

    metadata = {
        "synthetic": True,
        "schema_version": 1,
        "warning": "SYNTHETIC DEMO ONLY: not recorded from turtlesim, ROS nodes, or rosbag record.",
        "generator": "generate_demo_bag.py: exact ideal unicycle kinematics",
        "amplitude": amplitude,
        "amplitude_mode": amplitude_mode,
        "period_s": period,
        "linear_speed": linear_speed,
        "angular_speed_rad_s": angular_speed,
        "radius": radius,
        "prelude_rest_s": 2.0,
        "motion_duration_s": motion_duration,
        "postlude_rest_s": 1.0,
        "motion_cycles": 3,
        "command_rate_hz": 20,
        "command_schedule": "20 Hz plus the exact stop transition if it falls between regular samples",
        "pose_rate_hz": 60,
        "pose_sample_offset_s": 0.008,
        "sample_counts": counts,
        "fixed_synthetic_epoch_ns": ORIGIN_NS,
        "limitations": [
            "No ROS runtime was executed to create this bag.",
            "No turtlesim time steps, scheduler effects, transport delays or wall collisions are simulated.",
            "The bag verifies file-format reading and plotting, not a live ROS integration.",
        ],
    }
    # The declaration lives in the bag as well as its sidecar: moving or
    # renaming the bag alone must not turn synthetic samples into unlabeled data.
    with Writer(output) as writer:
        connections = {
            CMD_TOPIC: writer.add_connection(CMD_TOPIC, "geometry_msgs/msg/Twist", typestore=typestore),
            POSE_TOPIC: writer.add_connection(POSE_TOPIC, POSE_TYPE, typestore=typestore),
            PROVENANCE_TOPIC: writer.add_connection(PROVENANCE_TOPIC, "std_msgs/msg/String", typestore=typestore),
        }
        declaration = string_message(json.dumps(metadata, sort_keys=True, allow_nan=False))
        writer.write(connections[PROVENANCE_TOPIC], ORIGIN_NS, typestore.serialize_ros1(declaration, declaration.__msgtype__))
        for timestamp_ns, topic in events:
            time_s = (timestamp_ns - ORIGIN_NS) / 1e9
            x, y, theta, v, omega = ideal_state(time_s, linear_speed, angular_speed, motion_duration)
            if topic == CMD_TOPIC:
                if timestamp_ns >= stop_timestamp_ns:
                    v, omega = 0.0, 0.0
                message = twist(vector(v, 0.0, 0.0), vector(0.0, 0.0, omega))
            else:
                message = turtle_pose(x, y, theta, v, omega)
            writer.write(connections[topic], timestamp_ns, typestore.serialize_ros1(message, message.__msgtype__))
    checksum = hashlib.sha256()
    with output.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            checksum.update(chunk)
    metadata.update(bag_file=output.name, bag_sha256=checksum.hexdigest())
    with provenance.open("x", encoding="utf-8") as handle:
        handle.write(json.dumps(metadata, indent=2, allow_nan=False) + "\n")
    return metadata


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True, help="New synthetic .bag file; existing files are refused")
    parser.add_argument("--amplitude", type=float, default=1.0)
    parser.add_argument("--period", type=float, default=8.0, help="One revolution in seconds")
    parser.add_argument("--amplitude-mode", choices=("speed", "radius"), default="speed")
    args = parser.parse_args(argv)
    try:
        metadata = generate(args.output, args.amplitude, args.period, args.amplitude_mode)
    except (ImportError, OSError, ValueError) as exc:
        print("Synthetic bag generation failed: {}".format(exc), file=sys.stderr)
        return 1
    print(metadata["warning"])
    print("Created {} and {}".format(args.output.expanduser().resolve(), args.output.expanduser().with_suffix(".provenance.json").resolve()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
