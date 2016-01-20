#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
import math
from morse.testing.ros import RosTestCase
from morse.testing.testing import testlogger

import rospy
import std_msgs
import nav_msgs.msg # do not conflict with morse builder
from geometry_msgs.msg import Twist
from time import sleep

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def send_speed(s, v, w, t):
    msg = Twist()
    msg.linear.x = v
    msg.angular.z = w
    s.publish(msg)
    sleep(t)
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    s.publish(msg)

class DataStreamTest(RosTestCase):

    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = ATRV()

        odometry = Odometry()
        robot.append(odometry)
        odometry.add_stream('ros')

        motion = MotionVW()
        robot.append(motion)
        motion.add_stream('ros')
        
        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def pose_callback(self, data):
        self.pos = data

    def test_vw_controller(self):
        # XXX 
        # test the orientation part, but the easy way is to use numpy or
        # tf, and don't want to add too much dependency for test

        rospy.init_node('morse_ros_data_stream_test')
        rospy.Subscriber('/robot/odometry', nav_msgs.msg.Odometry, self.pose_callback)

        msg = rospy.client.wait_for_message('/robot/odometry', nav_msgs.msg.Odometry, timeout = 10)
        self.assertIsNotNone(msg)

        cmd_stream = rospy.Publisher('/robot/motion', Twist)
       
        self.assertTrue(hasattr(self, "pos"))
        precision=0.15

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)
 
        # sleep to make sure that the other peer can read it ...
        sleep(1)

        send_speed(cmd_stream, 1.0, 0.0, 2.0)
        sleep(1)

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 2.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)

        send_speed(cmd_stream, -1.0, 0.0, 2.0)

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)

        # sleep to make sure that the other peer can read it ...
        sleep(1)

        send_speed(cmd_stream, 1.0, -math.pi/4.0, 2.0)
        sleep(1)

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 4.0 / math.pi, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, -4.0 / math.pi, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)

        # sleep to make sure that the other peer can read it ...
        sleep(1)

        send_speed(cmd_stream, 0.5, -math.pi/8.0, 12.0)
        sleep(1)

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)

        send_speed(cmd_stream, -2.0, math.pi/2.0, 3.0)

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 4.0 / math.pi, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, -4.0 / math.pi, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(DataStreamTest, time_modes = [TimeStrategies.BestEffort])
