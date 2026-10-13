"""Physical invariants and malformed ROS parameter coverage, without ROS."""

import math
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from turtlesim_circle.motion import CircleParameters, finite_number


class MotionTest(unittest.TestCase):
    def test_speed_mode_revolution_and_radius(self):
        p = CircleParameters.from_mapping({"period": 8, "amplitude": 1.5})
        self.assertEqual(p.linear_speed, 1.5)
        self.assertAlmostEqual(p.angular_speed * p.period, 2 * math.pi)
        self.assertAlmostEqual(p.radius, 12 / (2 * math.pi))
        self.assertAlmostEqual(p.radius * abs(p.angular_speed), p.linear_speed)

    def test_radius_mode_revolution_and_speed(self):
        p = CircleParameters.from_mapping({
            "period": 8, "amplitude": 1.5, "amplitude_mode": "radius"})
        self.assertEqual(p.radius, 1.5)
        self.assertAlmostEqual(p.linear_speed, 3 * math.pi / 8)
        self.assertAlmostEqual(p.linear_speed * p.period, 2 * math.pi * p.radius)

    def test_direction_reverses_rotation_only(self):
        for mode in ("speed", "radius"):
            forward = CircleParameters.from_mapping({"amplitude_mode": mode})
            reverse = CircleParameters.from_mapping({"amplitude_mode": mode, "direction": -1})
            self.assertEqual(forward.linear_speed, reverse.linear_speed)
            self.assertEqual(forward.angular_speed, -reverse.angular_speed)
            self.assertEqual(forward.radius, reverse.radius)

    def test_period_change_has_mode_specific_effect(self):
        speed = CircleParameters.from_mapping({"period": 4, "amplitude_mode": "speed"})
        radius = CircleParameters.from_mapping({"period": 4, "amplitude_mode": "radius"})
        self.assertEqual(speed.linear_speed, 1)
        self.assertAlmostEqual(speed.radius, 2 / math.pi)
        self.assertEqual(radius.radius, 1)
        self.assertAlmostEqual(radius.linear_speed, math.pi / 2)

    def test_zero_amplitude_rotates_in_place(self):
        for mode in ("speed", "radius"):
            p = CircleParameters.from_mapping({"amplitude": 0, "amplitude_mode": mode})
            self.assertEqual(p.linear_speed, 0)
            self.assertEqual(p.radius, 0)
            self.assertGreater(p.angular_speed, 0)

    def test_invalid_parameters_are_rejected(self):
        bad_values = {
            "period": [0, -1, float("nan"), float("inf"), True, None, "bad", 1e-320],
            "amplitude": [-1, float("nan"), float("inf"), True, None, "bad"],
            "amplitude_mode": [None, "sine", "SPEED", 1, []],
            "direction": [0, 2, 0.5, True, None, "bad", float("nan"), float("inf")],
        }
        for name, values in bad_values.items():
            for value in values:
                with self.subTest(name=name, value=value):
                    with self.assertRaises(ValueError):
                        CircleParameters.from_mapping({name: value})

    def test_finite_inputs_with_overflow_are_rejected(self):
        for params in (
            {"amplitude_mode": "radius", "amplitude": 1e308, "period": 0.1},
            {"amplitude_mode": "speed", "amplitude": 1e308, "period": 100},
        ):
            with self.assertRaises(ValueError):
                CircleParameters.from_mapping(params)

    def test_numeric_strings_are_supported(self):
        p = CircleParameters.from_mapping({"period": "8", "amplitude": "1", "direction": "-1"})
        self.assertEqual(p.period, 8)
        self.assertEqual(p.direction, -1)

    def test_finite_double_commands_that_overflow_pose_are_rejected(self):
        for params in ({"period": 1e-40}, {"amplitude": 1e40}):
            with self.assertRaises(ValueError):
                CircleParameters.from_mapping(params)

    def test_timing_values_require_finite_nonnegative_numbers(self):
        self.assertEqual(finite_number("duration", 0), 0)
        for value in (-1, float("nan"), float("inf"), True):
            with self.assertRaises(ValueError):
                finite_number("duration", value)


if __name__ == "__main__":
    unittest.main()
