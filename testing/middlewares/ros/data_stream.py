#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
import math
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('nav_msgs'); 
roslib.load_manifest('geometry_msgs')
import rospy
import std_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from time import sleep
import subprocess
from morse.testing.testing import MorseTestCase, testlogger

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

class DataStreamTest(MorseTestCase):
    def setUpMw(self):
        try:
            self.roscore_process = subprocess.Popen(['roscore'])
        except OSError as ose:
            testlogger.error("Error while launching roscore ! Check you can run it from command-line\n")
            raise ose

    def tearDownMw(self):
        self.roscore_process.terminate()

    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = Robot('atrv')

        pose = Sensor('pose')
        robot.append(pose)
        pose.configure_mw('ros')

        motion = Actuator('v_omega')
        robot.append(motion)
        motion.configure_mw('ros')
        
        env = Environment('indoors-1/indoor-1')
        env.configure_service('socket')

    def pose_callback(self, data):
        self.pos = data

    def test_vw_controller(self):
        # XXX 
        # test the orientation part, but the easy way is to use numpy or
        # tf, and don't want to add too much dependency for test

        rospy.init_node('morse_ros_data_stream_test')
        rospy.Subscriber('ATRV/Pose', Odometry, self.pose_callback)

        msg = rospy.client.wait_for_message('ATRV/Pose', Odometry, timeout = 10)
        self.assertTrue(msg != None)

        cmd_stream = rospy.Publisher('ATRV/Motion_Controller', Twist)
       
        self.assertTrue(hasattr(self, "pos"))
        precision=0.15

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)
 
        # sleep to make sure that the other peer can read it ...
        sleep(5)

        send_speed(cmd_stream, 1.0, 0.0, 2.0)

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 2.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)

        send_speed(cmd_stream, -1.0, 0.0, 2.0)

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)

        send_speed(cmd_stream, 1.0, -math.pi/4.0, 2.0)

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 4.0 / math.pi, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, -4.0 / math.pi, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)

       # self.assertAlmostEqual(pose['x'], 4.0/ math.pi , delta=precision)
       # self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=precision)
       # self.assertAlmostEqual(pose['z'], 0.0, delta=precision)
       # self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=precision)
       # self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
       # self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)

        send_speed(cmd_stream, 0.5, -math.pi/8.0, 12.0)

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)

        send_speed(cmd_stream, -2.0, math.pi/2.0, 3.0)

        self.assertAlmostEqual(self.pos.pose.pose.position.x, 4.0 / math.pi, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.y, -4.0 / math.pi, delta=precision)
        self.assertAlmostEqual(self.pos.pose.pose.position.z, 0.0, delta=precision)
       # pose = pose_stream.get()
       # self.assertAlmostEqual(pose['x'], 4.0/ math.pi , delta=0.08)
       # self.assertAlmostEqual(pose['y'], -4.0/ math.pi , delta=0.08)
       # self.assertAlmostEqual(pose['z'], 0.0, delta=0.08)
       # self.assertAlmostEqual(pose['yaw'], -math.pi/2.0, delta=0.08)
       # self.assertAlmostEqual(pose['pitch'], 0.0, delta=0.08)
       # self.assertAlmostEqual(pose['roll'], 0.0, delta=0.08)

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(DataStreamTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

