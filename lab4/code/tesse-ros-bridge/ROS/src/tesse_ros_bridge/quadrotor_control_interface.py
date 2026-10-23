#!/usr/bin/env python3

import numpy as np
import copy
import threading
import time
import json

import rospy
import tf
import tf2_ros
from std_msgs.msg import Header, String
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import Imu, CameraInfo
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from mav_msgs.msg import Actuators
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    Point,
    PointStamped,
    TransformStamped,
    Quaternion,
    Twist,
    PoseWithCovarianceStamped,
    Vector3Stamped,
)
from ackermann_msgs.msg import AckermannDriveStamped
from rosgraph_msgs.msg import Clock
from cv_bridge import CvBridge, CvBridgeError

import tesse_ros_bridge.utils
from tesse_ros_bridge.noise_simulator import NoiseParams, NoiseSimulator

from tesse_ros_bridge.srv import (
    SceneRequestService,
    ObjectSpawnRequestService,
    RepositionRequestService,
)
from tesse_ros_bridge.msg import CollisionStats
from tesse_ros_bridge import brh_T_blh

from tesse.msgs import *
from tesse.env import *
from tesse.utils import *


class TesseQuadrotorControlInterface:
    def __init__(self):
        """This class provides a ROS interface for controlling TESSE quadrotor agents.
        ROS users can simply send propeller speeds command to predefined propeller speeds
        topic, and this interface will transmit the appropriate messages to TESSE simulator.

        Quadrotor agent is controlled by setting the speed of each propeller. The speeds are radians per second.
        """
        # Networking parameters
        self.sim_ip = rospy.get_param("~sim_ip", "127.0.0.1")
        self.self_ip = rospy.get_param("~self_ip", "127.0.0.1")
        self.use_broadcast = rospy.get_param("~use_broadcast", False)
        self.position_port = rospy.get_param("~position_port", 9000)
        self.metadata_port = rospy.get_param("~metadata_port", 9001)
        self.image_port = rospy.get_param("~image_port", 9002)
        self.udp_port = rospy.get_param("~udp_port", 9004)
        self.step_port = rospy.get_param("~step_port", 9005)
        self.scan_port = rospy.get_param("~lidar_port", 9006)
        self.scan_udp_port = rospy.get_param("~lidar_udp_port", 9007)

        # Topics
        self.props_speeds_topic = "quadrotor_control/rotor_speed_cmds"
        self.command_timeout = max(
            0.05, float(rospy.get_param("~command_timeout", 0.35))
        )
        self.watchdog_rate = max(
            1.0, float(rospy.get_param("~watchdog_rate", 20.0))
        )

        # Initialize the Env object to communicate with simulator
        self.env = Env(
            simulation_ip=self.sim_ip,
            own_ip=self.self_ip,
            position_port=self.position_port,
            metadata_port=self.metadata_port,
            image_port=self.image_port,
            step_port=self.step_port,
        )

        # TESSE holds the last rotor command. A ROS-time timer is insufficient
        # because /clock can stop with the simulator. This watchdog therefore
        # uses monotonic wall time and an independent thread.
        self._send_lock = threading.Lock()
        self._shutdown = threading.Event()
        self._last_command_monotonic = None
        self._failsafe_active = False
        self.applied_command_pub = rospy.Publisher(
            "/tesse/applied_rotor_speed_cmds", Actuators, queue_size=20
        )
        self.watchdog_event_pub = rospy.Publisher(
            "/tesse/rotor_watchdog_event", String, queue_size=20, latch=True
        )
        rospy.on_shutdown(self.shutdown)
        self._watchdog_thread = threading.Thread(
            target=self._watchdog_loop,
            name="tesse-rotor-watchdog",
        )
        self._watchdog_thread.daemon = True
        self._watchdog_thread.start()
        self.props_speeds_sub = rospy.Subscriber(
            self.props_speeds_topic,
            Actuators,
            self.props_control_cb,
            queue_size=1,
        )
        rospy.loginfo(
            "TESSE rotor watchdog enabled: timeout=%.3f s, rate=%.1f Hz",
            self.command_timeout,
            self.watchdog_rate,
        )

    def _publish_event(self, event, reason=""):
        payload = {
            "event": event,
            "reason": reason,
            "wall_monotonic_s": time.monotonic(),
            "ros_time_s": rospy.Time.now().to_sec(),
        }
        self.watchdog_event_pub.publish(String(data=json.dumps(payload, sort_keys=True)))

    def _send_speeds_locked(self, speeds, source):
        """Send and record one command while the caller holds _send_lock."""
        self.env.send(PropSpeeds(speeds[0], speeds[1], speeds[2], speeds[3]))
        applied = Actuators()
        applied.header.stamp = rospy.Time.now()
        applied.header.frame_id = source
        applied.angular_velocities = list(speeds)
        self.applied_command_pub.publish(applied)

    def _activate_failsafe(self, reason, force=False):
        """Send one explicit zero command when entering a fail-safe state."""
        with self._send_lock:
            # A callback may have been blocked in env.send when the watchdog
            # first observed an old timestamp.  Recheck after acquiring the
            # same send mutex so a fresh command is never overwritten by a
            # stale watchdog decision.
            if not force:
                last = self._last_command_monotonic
                if last is not None and time.monotonic() - last <= self.command_timeout:
                    return
            if self._failsafe_active and not force:
                return
            self._send_speeds_locked((0.0, 0.0, 0.0, 0.0), "watchdog_zero")
            self._failsafe_active = True
        self._publish_event("WATCHDOG_ZERO", reason)
        rospy.logerr("Rotor watchdog commanded zero speed: %s", reason)

    def _watchdog_loop(self):
        period = 1.0 / self.watchdog_rate
        while not self._shutdown.is_set() and not rospy.is_shutdown():
            now = time.monotonic()
            last = self._last_command_monotonic
            if last is None:
                self._activate_failsafe("no actuator command received")
            elif now - last > self.command_timeout:
                self._activate_failsafe(
                    "command age {:.3f} s exceeded {:.3f} s".format(
                        now - last, self.command_timeout
                    )
                )
            self._shutdown.wait(period)

    def shutdown(self):
        """Best-effort stop used for clean ROS shutdown."""
        self._shutdown.set()
        try:
            self._activate_failsafe("bridge shutdown", force=True)
            rospy.logwarn("TESSE bridge shutdown: zero rotor speed sent")
        except Exception as exception:
            rospy.logerr("Failed to send shutdown zero command: %s", exception)

    def props_control_cb(self, msg):
        """Callback function used for propeller speed control

        :param msg: A mav_msgs.msg.Actuators message. The field angular_velocities is used for setting the propeller speeds.
        """
        speeds = tuple(msg.angular_velocities[:4])
        if len(speeds) != 4 or not np.all(np.isfinite(speeds)):
            rospy.logerr_throttle(
                1.0, "Rejected invalid rotor command; entering fail-safe"
            )
            self._activate_failsafe("invalid actuator command", force=True)
            return

        with self._send_lock:
            self._send_speeds_locked(speeds, "controller_request")
            self._last_command_monotonic = time.monotonic()
            restored = self._failsafe_active
            self._failsafe_active = False
        if restored:
            rospy.loginfo("Rotor command stream restored; watchdog released")
            self._publish_event("STREAM_RESTORED", "fresh controller command")


if __name__ == "__main__":
    rospy.init_node("TesseQuadrotorControlInterface_node")
    node = TesseQuadrotorControlInterface()
    rospy.spin()
