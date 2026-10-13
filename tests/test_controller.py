"""Exercise controller transitions with an explicit fake clock and ROS stubs.

These are controller unit tests, not evidence of a live ROS/turtlesim run.
"""

import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
        self.angular = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


def pose(**updates):
    fields = dict(x=5.5, y=5.5, theta=0.0, linear_velocity=0.0, angular_velocity=0.0)
    fields.update(updates)
    return types.SimpleNamespace(**fields)


class Clock:
    def __init__(self):
        self.now = 0.0
        self.events = []

    def monotonic(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds
        for at, action in list(self.events):
            if at <= self.now + 1e-10:
                self.events.remove((at, action))
                action()


class FakeRos(types.ModuleType):
    ROSException = RuntimeError
    ROSInterruptException = RuntimeError

    def __init__(self, clock, params=None, stop_at=1.0):
        super().__init__("rospy")
        self.clock = clock
        self.params = {"~rate": 10, "~startup_delay": 0}
        self.params.update(params or {})
        self.stop_at = stop_at
        self.connections = 0
        self.recorder_connections = 1
        self.sent = []
        self.subscriptions = {}
        self.errors = []
        self.after_publish = lambda: None

    def get_param(self, name, default):
        return self.params.get(name, default)

    get_param_cached = get_param

    def Publisher(self, *_args, **_kwargs):
        return self

    def Subscriber(self, topic, _message_type, callback, **_kwargs):
        self.subscriptions[topic] = callback
        return types.SimpleNamespace(get_num_connections=lambda: self.recorder_connections)

    def on_shutdown(self, callback):
        self.shutdown_callback = callback

    def get_num_connections(self):
        return self.connections

    def is_shutdown(self):
        return self.clock.now >= self.stop_at

    def publish(self, command):
        self.sent.append((self.clock.now, command.linear.x, command.angular.z))
        self.after_publish()

    def loginfo(self, *_args):
        pass

    logwarn = loginfo
    loginfo_throttle = loginfo

    def logerr_throttle(self, *_args):
        self.errors.append(_args)


def load_with_stubs(fake):
    modules = {"rospy": fake}
    for package, message_name, message_class in (
        ("geometry_msgs", "Twist", Twist),
        ("std_msgs", "String", types.SimpleNamespace),
        ("turtlesim", "Pose", types.SimpleNamespace),
    ):
        modules[package] = types.ModuleType(package)
        message_module = types.ModuleType(package + ".msg")
        setattr(message_module, message_name, message_class)
        modules[package + ".msg"] = message_module
    specification = importlib.util.spec_from_file_location(
        "circle_controller_under_test", ROOT / "scripts" / "circle_controller.py")
    module = importlib.util.module_from_spec(specification)
    with patch.dict(sys.modules, modules):
        specification.loader.exec_module(module)
    module.time = fake.clock
    return module


class ControllerTest(unittest.TestCase):
    def make_controller(self, params=None, stop_at=1.0):
        clock = Clock()
        fake = FakeRos(clock, params, stop_at)
        module = load_with_stubs(fake)
        return module.CircleController(), fake, clock

    def test_waits_for_pose_subscriber_recorder_and_startup_delay(self):
        controller, fake, clock = self.make_controller({
            "~wait_for_recorder": True, "~startup_delay": 0.2})
        clock.events = [
            (0.1, lambda: controller._pose_received(pose())),
            (0.2, lambda: setattr(fake, "connections", 2)),
            (0.3, lambda: controller._recorder_started(types.SimpleNamespace(data="test.bag"))),
        ]
        controller.run()
        first_motion = next(at for at, speed, _ in fake.sent if speed > 0)
        self.assertGreaterEqual(first_motion + 1e-10, 0.5)
        self.assertLess(first_motion, 0.8)
        self.assertTrue(all(v == w == 0 for at, v, w in fake.sent if at < first_motion))

    def test_invalid_dynamic_parameter_stops_and_valid_update_recovers(self):
        controller, fake, _clock = self.make_controller(stop_at=0.6)
        controller._pose_received(pose())
        fake.connections = 1

        def update_parameters():
            if len(fake.sent) == 2:
                fake.params["~period"] = 0
            elif len(fake.sent) == 4:
                fake.params.update({"~period": 4, "~amplitude_mode": "radius", "~amplitude": 2})

        fake.after_publish = update_parameters
        controller.run()
        self.assertGreater(fake.sent[1][1], 0)
        self.assertTrue(all(v == w == 0 for _at, v, w in fake.sent[2:4]))
        self.assertTrue(fake.errors)
        self.assertAlmostEqual(fake.sent[4][1], math.pi)
        self.assertAlmostEqual(fake.sent[4][2], math.pi / 2)

    def test_duration_stops_and_keeps_publishing_zero(self):
        controller, fake, _clock = self.make_controller({"~duration": 0.25}, stop_at=0.9)
        controller._pose_received(pose())
        fake.connections = 1
        controller.run()
        moving = [index for index, (_at, speed, _w) in enumerate(fake.sent) if speed > 0]
        self.assertTrue(moving)
        after_motion = fake.sent[moving[-1] + 1:]
        self.assertGreaterEqual(len(after_motion), 3)
        self.assertTrue(all(v == w == 0 for _at, v, w in after_motion))

    def test_shutdown_sends_zero_burst_with_wall_clock(self):
        controller, fake, clock = self.make_controller()
        controller.stop()
        self.assertEqual(len(fake.sent), 5)
        self.assertTrue(all(v == w == 0 for _at, v, w in fake.sent))
        self.assertAlmostEqual(clock.now, 0.1)

    def test_low_rate_is_rejected_before_starting(self):
        with self.assertRaises(ValueError):
            self.make_controller({"~rate": 1})

    def test_recorder_announcement_does_not_bypass_velocity_connections(self):
        controller, fake, clock = self.make_controller({"~wait_for_recorder": True}, stop_at=0.6)
        controller._pose_received(pose())
        controller._recorder_started(types.SimpleNamespace(data="test.bag"))
        fake.connections = 1
        clock.events = [(0.3, lambda: setattr(fake, "connections", 2))]
        controller.run()
        first_motion = next(at for at, speed, _ in fake.sent if speed > 0)
        self.assertGreaterEqual(first_motion, 0.3 - 1e-10)
        self.assertLess(first_motion, 0.5)

    def test_stale_pose_stops_despite_remaining_subscribers_and_recovery_waits(self):
        controller, fake, clock = self.make_controller(
            {"~pose_timeout": 0.15, "~startup_delay": 0.09}, stop_at=0.7)
        controller._pose_received(pose())
        fake.connections = 2  # a recorder or plotter can remain after turtlesim stops
        clock.events = [(0.4, lambda: controller._pose_received(pose()))]
        controller.run()
        self.assertTrue(any(v > 0 for at, v, _w in fake.sent if at < 0.2))
        self.assertTrue(all(v == w == 0 for at, v, w in fake.sent if 0.2 <= at < 0.49))
        self.assertTrue(any(v > 0 for at, v, _w in fake.sent if 0.49 <= at < 0.59))

    def test_invalid_pose_stops_without_waiting_for_timeout(self):
        controller, fake, clock = self.make_controller(stop_at=0.6)
        controller._pose_received(pose())
        fake.connections = 1
        clock.events = [(0.2, lambda: controller._pose_received(pose(x=float("nan"))))]
        controller.run()
        self.assertTrue(any(v > 0 for at, v, _w in fake.sent if at < 0.2))
        self.assertTrue(all(v == w == 0 for at, v, w in fake.sent if at >= 0.2))

    def test_disconnected_recorder_requires_a_new_open_bag_announcement(self):
        controller, fake, clock = self.make_controller({"~wait_for_recorder": True}, stop_at=0.8)
        controller._pose_received(pose())
        controller._recorder_started(types.SimpleNamespace(data="first.bag"))
        fake.connections = 2
        clock.events = [
            (0.2, lambda: setattr(fake, "recorder_connections", 0)),
            (0.4, lambda: setattr(fake, "recorder_connections", 1)),
            (0.6, lambda: controller._recorder_started(types.SimpleNamespace(data="second.bag"))),
        ]
        controller.run()
        self.assertTrue(any(v > 0 for at, v, _w in fake.sent if at < 0.2))
        self.assertTrue(all(v == w == 0 for at, v, w in fake.sent if 0.2 <= at < 0.59))
        self.assertTrue(any(v > 0 for at, v, _w in fake.sent if 0.59 <= at < 0.8))

    def test_unexpected_loop_failure_sends_a_stop_burst(self):
        controller, fake, _clock = self.make_controller()
        controller._pose_received(pose())
        fake.connections = 1

        def fail_parameters():
            raise RuntimeError("simulated transport failure")

        fake.after_publish = lambda: setattr(controller, "_parameters", fail_parameters)
        with self.assertRaisesRegex(RuntimeError, "simulated transport failure"):
            controller.run()
        self.assertGreater(fake.sent[0][1], 0)
        self.assertEqual(len(fake.sent), 6)
        self.assertTrue(all(v == w == 0 for _at, v, w in fake.sent[1:]))

    def test_stop_during_command_preparation_cannot_publish_a_new_nonzero_command(self):
        controller, fake, _clock = self.make_controller()
        controller._pose_received(pose())
        fake.connections = 1
        original = controller._parameters

        def stop_during_read():
            controller.stop()
            return original()

        controller._parameters = stop_during_read
        controller.run()
        self.assertTrue(fake.sent)
        self.assertTrue(all(v == w == 0 for _at, v, w in fake.sent))

    def test_motion_does_not_resume_after_duration_expires_during_outage(self):
        controller, fake, clock = self.make_controller(
            {"~duration": 0.25, "~pose_timeout": 0.15}, stop_at=0.8)
        controller._pose_received(pose())
        fake.connections = 1
        clock.events = [(0.5, lambda: controller._pose_received(pose()))]
        controller.run()
        self.assertTrue(any(v > 0 for _at, v, _w in fake.sent))
        self.assertTrue(all(v == w == 0 for at, v, w in fake.sent if at >= 0.2))

    def test_pose_timeout_requires_a_finite_positive_number(self):
        for value in (0, -1, True, float("nan"), float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    self.make_controller({"~pose_timeout": value})


if __name__ == "__main__":
    unittest.main()
