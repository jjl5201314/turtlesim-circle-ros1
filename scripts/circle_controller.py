#!/usr/bin/env python3
"""Publish circle commands with live, validated ROS private parameters."""

import threading
import time
import math

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from turtlesim.msg import Pose

from turtlesim_circle.motion import CircleParameters, finite_number


class CircleController:
    def __init__(self):
        self.rate = finite_number("rate", rospy.get_param("~rate", 30.0), positive=True)
        if not 2.0 <= self.rate <= 1000.0:
            raise ValueError("rate must be between 2 and 1000 Hz")
        self.duration = finite_number("duration", rospy.get_param("~duration", 0.0))
        self.startup_delay = finite_number(
            "startup_delay", rospy.get_param("~startup_delay", 2.0))
        self.pose_timeout = finite_number(
            "pose_timeout", rospy.get_param("~pose_timeout", 1.0), positive=True)
        self.wait_for_recorder = rospy.get_param("~wait_for_recorder", False)
        if not isinstance(self.wait_for_recorder, bool):
            raise ValueError("wait_for_recorder must be a boolean")
        self.last_pose_time = None
        self.recorder_ready = threading.Event()
        self.recorder_lock = threading.Lock()
        self.stopping = threading.Event()
        self.publish_lock = threading.RLock()
        self.publisher = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=1)
        self.pose_subscriber = rospy.Subscriber(
            "/turtle1/pose", Pose, self._pose_received, queue_size=1)
        self.recorder_subscriber = None
        if self.wait_for_recorder:
            self.recorder_subscriber = rospy.Subscriber(
                "/circle_recorder/begin_write", String,
                self._recorder_started, queue_size=1)
        rospy.on_shutdown(self.stop)

    def _pose_received(self, message):
        if all(math.isfinite(value) for value in (
                message.x, message.y, message.theta,
                message.linear_velocity, message.angular_velocity)):
            self.last_pose_time = time.monotonic()
        else:
            # An invalid pose must stop motion immediately, even if the last
            # valid pose arrived less than pose_timeout seconds ago.
            self.last_pose_time = None

    def _recorder_started(self, message):
        with self.recorder_lock:
            if not self.recorder_ready.is_set():
                rospy.loginfo("Recorder opened %s", message.data)
            self.recorder_ready.set()

    def _parameters(self):
        return CircleParameters.from_mapping({
            "period": rospy.get_param_cached("~period", 8.0),
            "amplitude": rospy.get_param_cached("~amplitude", 1.0),
            "amplitude_mode": rospy.get_param_cached("~amplitude_mode", "speed"),
            "direction": rospy.get_param_cached("~direction", 1),
        })

    def _ready(self, now):
        if self.wait_for_recorder:
            with self.recorder_lock:
                if self.recorder_subscriber.get_num_connections() == 0:
                    # A replacement recorder must announce its own open bag.
                    self.recorder_ready.clear()
                recording_ready = self.recorder_ready.is_set()
            if not recording_ready:
                return False
        pose_time = self.last_pose_time
        if pose_time is None or now - pose_time > self.pose_timeout:
            return False
        # begin_write announces an open bag, not a completed TCP handshake.
        # The standard launch needs both turtlesim and rosbag attached.
        minimum_connections = 2 if self.wait_for_recorder else 1
        if self.publisher.get_num_connections() < minimum_connections:
            return False
        return True

    def run(self):
        try:
            self._run()
        finally:
            # Also stop after an unexpected parameter/transport exception,
            # not only after ROS has invoked its normal shutdown callbacks.
            self.stop()

    def _run(self):
        interval = 1.0 / self.rate
        ready_since = None
        motion_start = None
        previous = None
        finished = False
        deadline = time.monotonic()
        while not rospy.is_shutdown() and not self.stopping.is_set():
            now = time.monotonic()
            command = Twist()
            if not self._ready(now):
                ready_since = None
                rospy.loginfo_throttle(
                    5.0, "Stopped: waiting for fresh turtle pose, velocity subscribers, and optional recorder")
            elif ready_since is None:
                ready_since = now
            ready_to_move = (ready_since is not None and
                             now - ready_since >= self.startup_delay)
            if motion_start is None and ready_to_move:
                motion_start = now
                rospy.loginfo("Startup complete; circle motion window begins")
            if motion_start is not None:
                if self.duration > 0.0 and now - motion_start >= self.duration:
                    if not finished:
                        rospy.loginfo("Duration elapsed; publishing zero velocity until shutdown")
                        finished = True
                elif not finished and ready_to_move:
                    try:
                        parameters = self._parameters()
                        command.linear.x = parameters.linear_speed
                        command.angular.z = parameters.angular_speed
                        if parameters != previous:
                            rospy.loginfo(
                                "mode=%s T=%.3f A=%.3f direction=%d -> v=%.3f w=%.3f R=%.3f",
                                parameters.amplitude_mode, parameters.period,
                                parameters.amplitude, parameters.direction,
                                parameters.linear_speed, parameters.angular_speed,
                                parameters.radius)
                            if parameters.radius > 2.5:
                                rospy.logwarn(
                                    "Radius > 2.5 may hit a turtlesim wall from the default pose")
                            previous = parameters
                    except (ValueError, TypeError, KeyError) as exc:
                        rospy.logerr_throttle(2.0, "Invalid circle parameter; stopped: %s", exc)
            with self.publish_lock:
                # stop() can run from a ROS callback while this loop is active.
                self.publisher.publish(Twist() if self.stopping.is_set() else command)
            # Turtlesim runs in real time, not /clock time. This also keeps a
            # stop responsive when an unrelated /use_sim_time setting is stale.
            deadline += interval
            delay = deadline - time.monotonic()
            if delay > 0.0:
                time.sleep(delay)
            else:
                deadline = time.monotonic()

    def stop(self):
        self.stopping.set()
        # Best effort: the subscriber/recorder may also be shutting down.
        # Wall sleep is intentional; rospy.sleep may raise during shutdown.
        for _ in range(5):
            try:
                with self.publish_lock:
                    self.publisher.publish(Twist())
            except rospy.ROSException:
                break
            time.sleep(0.02)


def main():
    rospy.init_node("circle_controller")
    try:
        controller = CircleController()
        controller.run()
    except (ValueError, TypeError) as exc:
        rospy.logfatal("Controller configuration error: %s", exc)
        return 1
    except rospy.ROSInterruptException:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
