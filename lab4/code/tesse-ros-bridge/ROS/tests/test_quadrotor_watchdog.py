#!/usr/bin/env python3

"""Regression test for the TESSE rotor-command watchdog.

The test replaces the network environment and ROS plumbing with small fakes.  It
therefore verifies the safety-state transitions without requiring a running
TESSE process or opening UDP sockets.
"""

import time
import unittest
from types import SimpleNamespace
from unittest import mock

import tesse_ros_bridge.quadrotor_control_interface as bridge_module


class FakeEnv:
    """Capture commands that would otherwise be sent to TESSE."""

    latest = None

    def __init__(self, **_kwargs):
        self.commands = []
        FakeEnv.latest = self

    def send(self, command):
        self.commands.append(tuple(command))


def fake_get_param(name, default):
    if name.endswith("command_timeout"):
        return 0.05
    if name.endswith("watchdog_rate"):
        return 100.0
    return default


class QuadrotorWatchdogTest(unittest.TestCase):
    def test_zero_restore_and_timeout_sequence(self):
        with mock.patch.object(bridge_module, "Env", FakeEnv), \
                mock.patch.object(
                    bridge_module,
                    "PropSpeeds",
                    side_effect=lambda a, b, c, d: (a, b, c, d),
                ), \
                mock.patch.object(
                    bridge_module.rospy, "get_param", side_effect=fake_get_param
                ), \
                mock.patch.object(bridge_module.rospy, "Subscriber"), \
                mock.patch.object(bridge_module.rospy, "Publisher"), \
                mock.patch.object(
                    bridge_module.rospy.Time,
                    "now",
                    return_value=bridge_module.rospy.Time(1.0),
                ), \
                mock.patch.object(bridge_module.rospy, "on_shutdown"), \
                mock.patch.object(
                    bridge_module.rospy, "is_shutdown", return_value=False
                ), \
                mock.patch.object(bridge_module.rospy, "loginfo"), \
                mock.patch.object(bridge_module.rospy, "logwarn"), \
                mock.patch.object(bridge_module.rospy, "logerr"), \
                mock.patch.object(bridge_module.rospy, "logerr_throttle"):
            node = bridge_module.TesseQuadrotorControlInterface()
            try:
                time.sleep(0.03)
                self.assertEqual(FakeEnv.latest.commands[-1], (0.0, 0.0, 0.0, 0.0))

                node.props_control_cb(
                    SimpleNamespace(angular_velocities=[11.0, 12.0, 13.0, 14.0])
                )
                self.assertEqual(FakeEnv.latest.commands[-1], (11.0, 12.0, 13.0, 14.0))

                time.sleep(0.09)
                self.assertEqual(FakeEnv.latest.commands[-1], (0.0, 0.0, 0.0, 0.0))
            finally:
                node.shutdown()


if __name__ == "__main__":
    unittest.main()
