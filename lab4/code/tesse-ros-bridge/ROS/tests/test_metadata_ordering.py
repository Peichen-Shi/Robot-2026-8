#!/usr/bin/env python3
"""Execute the real metadata callback guard without a simulator connection."""
import unittest
from unittest.mock import patch
from tesse_ros_bridge.tesse_ros_node import TesseROSWrapper


class OrderingTests(unittest.TestCase):
    def setUp(self):
        self.node=object.__new__(TesseROSWrapper)
        self.node.publish_metadata=False
        self.node.prev_time=10.

    def test_duplicate_and_old_packets_do_not_kill_receiver(self):
        for stamp in (10.,9.98):
            with patch('tesse_ros_bridge.utils.parse_metadata',return_value={'time':stamp}), \
                 patch('tesse_ros_bridge.utils.process_metadata') as process, \
                 patch('rospy.logwarn_throttle'):
                self.node.meta_cb(b'packet')
                process.assert_not_called()
                self.assertEqual(self.node.prev_time,10.)

    def test_new_packet_reaches_processing(self):
        self.node.prev_vel_brh=None
        self.node.prev_enu_R_brh=None
        with patch('tesse_ros_bridge.utils.parse_metadata',return_value={'time':10.02}), \
             patch('tesse_ros_bridge.utils.process_metadata',side_effect=RuntimeError('processing reached')):
            with self.assertRaisesRegex(RuntimeError,'processing reached'):
                self.node.meta_cb(b'packet')


if __name__=='__main__': unittest.main()
