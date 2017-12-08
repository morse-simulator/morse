#! /usr/bin/env python
"""
This script tests the Sick laser scanner with ROS in MORSE.
"""

import sys
import time
import math
import struct
from morse.testing.ros import RosTestCase
from morse.testing.testing import testlogger

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Include this import to be able to use your test file as a regular
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def pub_vw(topic, v, w):
    pub = rospy.Publisher(topic, Twist)
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = w
    # wait 1 second for publisher
    rospy.sleep(1.0)
    pub.publish(msg)

class SickLaserRosTest(RosTestCase):

    def setUpEnv(self):
        """ Defines the test scenario """

        robot = ATRV()

        motion = MotionVW()
        robot.append(motion)
        motion.add_stream('ros')

        sick = Sick()
        sick.translate(x = 0.18, z = 0.94)
        robot.append(sick)
        # sick.properties(scan_window = 270, resolution = .25)
        sick.properties(scan_window = 180, resolution = 1)
        sick.add_stream('ros')
        # test does not call sick.__del__() so create laser arc manually
        sick.create_laser_arc()

        env = Environment('indoors-1/boxes', fastmode=True)

    def test_sick_laser(self):
        rospy.init_node('morse_ros_laser_testing')

        motion_topic = '/robot/motion'
        laser_topic  = '/robot/sick'

        pub_vw(motion_topic, 1, 1)

        old = []
        for step in range(5):
            msg = rospy.wait_for_message(laser_topic, LaserScan, 10)

            self.assertEqual(len(msg.ranges), 181) # 180 + center ray
            self.assertNotEqual(msg.ranges, old)
            old = msg.ranges
            # assert that : near <= distance <= far
            for distance in msg.ranges:
                self.assertGreaterEqual(distance, 0.1)
                self.assertLessEqual(distance, 30)

            time.sleep(0.2) # wait for turning

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(SickLaserRosTest, time_modes = [TimeStrategies.BestEffort])
