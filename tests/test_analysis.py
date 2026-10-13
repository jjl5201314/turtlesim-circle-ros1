"""Behavior tests for asynchronous timestamps, validation and ROS1 bag reading."""

import csv
import contextlib
import io
import importlib.util
import json
import tempfile
import unittest
from unittest import mock
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np

SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "analyze_bag.py"
SPEC = importlib.util.spec_from_file_location("analyze_bag", SCRIPT)
analysis = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = analysis
SPEC.loader.exec_module(analysis)
GENERATOR_SPEC = importlib.util.spec_from_file_location("generate_demo_bag", SCRIPT.with_name("generate_demo_bag.py"))
generator = importlib.util.module_from_spec(GENERATOR_SPEC)
GENERATOR_SPEC.loader.exec_module(generator)
CMD_TOPIC = "/turtle1/cmd_vel"
POSE_TOPIC = "/turtle1/pose"


def command(linear=1.5, angular=0.5):
    return SimpleNamespace(
        linear=SimpleNamespace(x=linear, y=0.0, z=0.0),
        angular=SimpleNamespace(x=0.0, y=0.0, z=angular),
    )


def pose(x=5.0, y=5.0, theta=0.0):
    return SimpleNamespace(x=x, y=y, theta=theta, linear_velocity=1.5, angular_velocity=0.5)


class AnalysisTests(unittest.TestCase):
    def test_shared_origin_keeps_asynchronous_offset_and_nanosecond_precision(self):
        epoch = 1_700_000_000_000_000_000
        recording = analysis.collect_samples([
            (CMD_TOPIC, epoch + 100_000_001, command()),
            (POSE_TOPIC, epoch, pose()),
            (CMD_TOPIC, epoch + 50_000_001, command()),
            (POSE_TOPIC, epoch + 16_000_000, pose()),
        ], CMD_TOPIC, POSE_TOPIC)
        self.assertEqual(recording.origin_ns, epoch)
        np.testing.assert_allclose(
            analysis.relative_seconds(recording.command, recording.origin_ns),
            [0.050000001, 0.100000001], atol=1e-12, rtol=0,
        )
        np.testing.assert_allclose(
            analysis.relative_seconds(recording.pose, recording.origin_ns), [0, 0.016]
        )

    def test_missing_stream_and_nonfinite_values_fail_explicitly(self):
        with self.assertRaisesRegex(analysis.AnalysisError, "contains no samples"):
            analysis.collect_samples([(CMD_TOPIC, 0, command())], CMD_TOPIC, POSE_TOPIC)
        with self.assertRaisesRegex(analysis.AnalysisError, "Non-finite"):
            analysis.collect_samples([(CMD_TOPIC, 0, command(float("nan")))], CMD_TOPIC, POSE_TOPIC)

    def test_topic_validation_accepts_normalized_names_and_rejects_wrong_type(self):
        analysis.validate_topics({CMD_TOPIC: {"geometry_msgs/msg/Twist"}, POSE_TOPIC: {"turtlesim/msg/Pose"}}, CMD_TOPIC, POSE_TOPIC)
        with self.assertRaisesRegex(analysis.AnalysisError, "must contain"):
            analysis.validate_topics({CMD_TOPIC: {"std_msgs/String"}, POSE_TOPIC: {"turtlesim/Pose"}}, CMD_TOPIC, POSE_TOPIC)
        with self.assertRaisesRegex(analysis.AnalysisError, "Missing topic"):
            analysis.validate_topics({CMD_TOPIC: {"geometry_msgs/Twist"}}, CMD_TOPIC, POSE_TOPIC)

    def test_csv_preserves_timestamp_and_unwrap_crossing(self):
        epoch = 1_700_000_000_000_000_000
        recording = analysis.collect_samples([
            (CMD_TOPIC, epoch, command()),
            (POSE_TOPIC, epoch, pose(theta=3.0)),
            (POSE_TOPIC, epoch + 1_000_000_000, pose(x=6.0, theta=-3.0)),
        ], CMD_TOPIC, POSE_TOPIC)
        summary = analysis.build_summary(recording, "test.bag")
        self.assertAlmostEqual(summary["pose"]["unwrapped_heading_change_rad"], 2 * np.pi - 6)
        self.assertAlmostEqual(summary["pose"]["sampled_path_length"], 1.0)
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "pose.csv"
            analysis.write_csv(destination, recording.pose, analysis.POSE_FIELDS, recording.origin_ns)
            with destination.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(rows[0]["bag_time_ns"], str(epoch))
            self.assertEqual(float(rows[1]["time_s"]), 1.0)
            self.assertEqual(float(rows[1]["theta"]), -3.0)

    def test_single_sample_summary_has_no_fabricated_rate(self):
        recording = analysis.collect_samples([(CMD_TOPIC, 1, command()), (POSE_TOPIC, 1, pose())], CMD_TOPIC, POSE_TOPIC)
        summary = analysis.build_summary(recording, "single.bag")
        self.assertEqual(summary["selected_duration_s"], 0)
        self.assertIsNone(summary["pose"]["max_sample_gap_s"])
        self.assertEqual(summary["pose"]["sampled_path_length"], 0)
        json.dumps(summary, allow_nan=False)

    def test_refuse_overwrite_preserves_existing_outputs_and_unrelated_files(self):
        recording = analysis.collect_samples([(CMD_TOPIC, 1, command()), (POSE_TOPIC, 1, pose())], CMD_TOPIC, POSE_TOPIC)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            (output / "pose.csv").write_text("original data", encoding="utf-8")
            (output / "notes.txt").write_text("user notes", encoding="utf-8")
            with self.assertRaisesRegex(analysis.AnalysisError, "--overwrite"):
                analysis.export_analysis(recording, output, "test.bag")
            self.assertEqual((output / "pose.csv").read_text(encoding="utf-8"), "original data")
            self.assertFalse((output / "cmd_vel.csv").exists())
            analysis.export_analysis(recording, output, "test.bag", overwrite=True)
            self.assertEqual((output / "notes.txt").read_text(encoding="utf-8"), "user notes")
            self.assertNotEqual((output / "pose.csv").read_text(encoding="utf-8"), "original data")

    def test_plot_failure_does_not_replace_old_report(self):
        recording = analysis.collect_samples([(CMD_TOPIC, 1, command()), (POSE_TOPIC, 1, pose())], CMD_TOPIC, POSE_TOPIC)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            for filename in analysis.ARTIFACTS:
                (output / filename).write_bytes(b"old report")
            with mock.patch.object(analysis, "write_figures", side_effect=ImportError("missing matplotlib")):
                with self.assertRaisesRegex(ImportError, "missing matplotlib"):
                    analysis.export_analysis(recording, output, "test.bag", overwrite=True)
            for filename in analysis.ARTIFACTS:
                self.assertEqual((output / filename).read_bytes(), b"old report")
            self.assertEqual(len(list(output.iterdir())), len(analysis.ARTIFACTS))

    def test_invalid_and_unfinalized_bags_fail_before_writing_outputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            active_bag = root / "recording.bag.active"
            active_bag.write_bytes(b"#ROSBAG V2.0\n")
            with self.assertRaisesRegex(analysis.AnalysisError, "finalized"):
                analysis.read_recording(active_bag)
            invalid_bag = root / "not_a_bag.bag"
            invalid_bag.write_bytes(b"this is not a bag")
            errors = io.StringIO()
            with contextlib.redirect_stderr(errors):
                status = analysis.main(["--bag", str(invalid_bag), "--output-dir", str(root / "report")])
            self.assertEqual(status, 1)
            self.assertIn("not a ROS1 bag", errors.getvalue())
            self.assertFalse((root / "report").exists())


try:
    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_types_from_msg, get_typestore
    HAS_ROSBAGS = True
except ImportError:
    HAS_ROSBAGS = False


@unittest.skipUnless(HAS_ROSBAGS, "rosbags is required for the real bag format round-trip test")
class RosbagRoundTripTests(unittest.TestCase):
    def test_synthetic_generator_provenance_and_three_complete_turns(self):
        with tempfile.TemporaryDirectory() as directory:
            bag_path = Path(directory) / "synthetic.bag"
            metadata = generator.generate(bag_path, amplitude=1.0, period=8.0, amplitude_mode="radius")
            self.assertTrue(metadata["synthetic"])
            self.assertAlmostEqual(metadata["radius"], 1.0)
            self.assertAlmostEqual(metadata["linear_speed"], np.pi / 4)
            recording = analysis.read_recording(bag_path, backend="rosbags")
            summary = analysis.build_summary(recording, bag_path)
            self.assertTrue(summary["synthetic"])
            self.assertEqual(summary["provenance_source"], "embedded bag declaration")
            self.assertEqual(summary["bag_file"], bag_path.name)
            self.assertEqual(summary["bag_sha256"], metadata["bag_sha256"])
            self.assertAlmostEqual(summary["pose"]["unwrapped_heading_change_rad"], 6 * np.pi, places=5)
            self.assertAlmostEqual(summary["pose"]["start_to_end_distance"], 0.0, places=5)
            self.assertEqual(recording.command.values[0, 0], 0.0)
            self.assertEqual(recording.command.values[-1, 0], 0.0)
            with self.assertRaises(FileExistsError):
                generator.generate(bag_path)

    def test_embedded_provenance_survives_bag_rename_without_sidecar(self):
        with tempfile.TemporaryDirectory() as directory:
            bag_path = Path(directory) / "demo.bag"
            generator.generate(bag_path)
            renamed = bag_path.with_name("renamed.bag")
            bag_path.rename(renamed)
            recording = analysis.read_recording(renamed, backend="rosbags")
            summary = analysis.build_summary(recording, renamed)
            self.assertTrue(summary["synthetic"])
            self.assertIsNone(summary["provenance_file"])
            self.assertEqual(summary["provenance_source"], "embedded bag declaration")

    def test_sidecar_for_different_bag_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            bag_path = Path(directory) / "demo.bag"
            metadata = generator.generate(bag_path)
            metadata["bag_sha256"] = "0" * 64
            bag_path.with_suffix(".provenance.json").write_text(json.dumps(metadata), encoding="utf-8")
            recording = analysis.read_recording(bag_path, backend="rosbags")
            with self.assertRaisesRegex(analysis.AnalysisError, "SHA-256 does not match"):
                analysis.build_summary(recording, bag_path)

    def test_generation_is_reproducible_and_custom_stop_command_is_zero(self):
        with tempfile.TemporaryDirectory() as directory:
            first = Path(directory) / "first.bag"
            second = Path(directory) / "second.bag"
            period = 0.1234567897
            first_metadata = generator.generate(first, period=period)
            second_metadata = generator.generate(second, period=period)
            self.assertEqual(first_metadata["bag_sha256"], second_metadata["bag_sha256"])
            recording = analysis.read_recording(first, backend="rosbags")
            stop_time = generator.ORIGIN_NS + round((2.0 + 3 * period) * 1e9)
            self.assertTrue(np.any(recording.command.timestamp_ns == stop_time))
            np.testing.assert_array_equal(recording.command.values[recording.command.timestamp_ns >= stop_time], 0)

    def test_generator_rejects_unbounded_or_nonfinite_parameters_without_files(self):
        with tempfile.TemporaryDirectory() as directory:
            bag_path = Path(directory) / "invalid.bag"
            for values in ({"period": 2000.0}, {"period": 1e-300}, {"amplitude": float("nan")}, {"amplitude": True}):
                with self.subTest(values=values), self.assertRaises(ValueError):
                    generator.generate(bag_path, **values)
                self.assertFalse(bag_path.exists())

    def test_truncated_bag_has_actionable_error(self):
        with tempfile.TemporaryDirectory() as directory:
            bag_path = Path(directory) / "truncated.bag"
            bag_path.write_bytes(b"#ROSBAG V2.0\n")
            with self.assertRaisesRegex(analysis.AnalysisError, "Ensure recording was stopped cleanly"):
                analysis.read_recording(bag_path, backend="rosbags")

    def test_read_and_export_real_ros1_format_bag(self):
        """Synthetic serialization fixture, not a claimed live turtlesim run."""
        typestore = get_typestore(Stores.ROS1_NOETIC)
        pose_type = "turtlesim/msg/Pose"
        typestore.register(get_types_from_msg(
            "float32 x\nfloat32 y\nfloat32 theta\nfloat32 linear_velocity\nfloat32 angular_velocity\n",
            pose_type,
        ))
        vector = typestore.types["geometry_msgs/msg/Vector3"]
        twist = typestore.types["geometry_msgs/msg/Twist"]
        turtle_pose = typestore.types[pose_type]
        with tempfile.TemporaryDirectory() as directory:
            bag_path = Path(directory) / "fixture.bag"
            with Writer(bag_path) as writer:
                cmd_connection = writer.add_connection(CMD_TOPIC, "geometry_msgs/msg/Twist", typestore=typestore)
                pose_connection = writer.add_connection(POSE_TOPIC, pose_type, typestore=typestore)
                for index in range(4):
                    timestamp = 1_700_000_000_000_000_000 + index * 100_000_000
                    msg = twist(vector(1.0, 0.0, 0.0), vector(0.0, 0.0, 0.5))
                    writer.write(cmd_connection, timestamp, typestore.serialize_ros1(msg, msg.__msgtype__))
                    msg = turtle_pose(5.0 + index * 0.1, 5.0, index * 0.05, 1.0, 0.5)
                    writer.write(pose_connection, timestamp + 10_000_000, typestore.serialize_ros1(msg, msg.__msgtype__))
            recording = analysis.read_recording(bag_path, backend="rosbags")
            self.assertEqual(len(recording.command.timestamp_ns), 4)
            self.assertEqual(len(recording.pose.timestamp_ns), 4)
            np.testing.assert_allclose(analysis.relative_seconds(recording.pose, recording.origin_ns), [0.01, 0.11, 0.21, 0.31])
            output = Path(directory) / "analysis"
            summary = analysis.export_analysis(recording, output, bag_path)
            for filename in summary["artifacts"]:
                self.assertTrue((output / filename).is_file(), filename)
                self.assertGreater((output / filename).stat().st_size, 0, filename)
            with (output / "pose.csv").open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(len(rows), 4)
            self.assertEqual(float(rows[0]["time_s"]), 0.01)
            saved = json.loads((output / "summary.json").read_text(encoding="utf-8"))
            self.assertEqual(saved["reader_backend"], "rosbags")
            self.assertAlmostEqual(saved["selected_duration_s"], 0.31)


if __name__ == "__main__":
    unittest.main()
