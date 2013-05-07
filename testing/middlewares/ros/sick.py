#! /usr/bin/env python
"""
This script tests the Sick laser scanner with ROS in MORSE.
"""

import sys
import time
import math
from morse.testing.ros import RosTestCase
from morse.testing.testing import testlogger

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('nav_msgs')
roslib.load_manifest('sensor_msgs')
roslib.load_manifest('geometry_msgs')
import rospy
import nav_msgs.msg # do not conflict with morse builder
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

# Include this import to be able to use your test file as a regular
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class SickLaserTest(RosTestCase):

    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        robot = ATRV('ATRV')

        motion = MotionVW('MotionVW')
        robot.append(motion)
        motion.add_stream('ros')

        odometry = Odometry('Odometry')
        odometry.translate(z=0.73)
        robot.append(odometry)
        odometry.add_stream('ros')

        sick = Sick('Sick')
        sick.translate(x = 0.18, z = 0.94)
        robot.append(sick)
        # sick.properties(scan_window = 270, resolution = .25)
        sick.properties(scan_window = 180, resolution = 1)
        sick.add_stream('ros')
        # test does not call sick.__del__() so create laser arc manually
        sick.create_laser_arc()

        env = Environment('indoors-1/boxes', fastmode=True)
        env.add_service('socket')

    def send_speed(self, v, w, t):
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.cmd_stream.publish(msg)
        time.sleep(t)
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.cmd_stream.publish(msg)

    def sensor_callback(self, message):
        self.sensor_message = message

    def subscribe_and_wait_for_message(self, topic, topic_type, timeout=30):
        if hasattr(self, 'sensor_sub'):
            self.sensor_sub.unregister()
        self.sensor_sub = rospy.Subscriber(topic, topic_type, self.sensor_callback)
        return self.wait_for_message(timeout)

    def wait_for_message(self, timeout=10):
        self.sensor_message = None
        timeout_t = time.time() + timeout
        while not self.sensor_message and timeout_t > time.time():
            time.sleep(.1)
        return self.sensor_message

    def init_sensor_test(self, topic, topic_type):
        rospy.init_node('morse_testing', log_level=rospy.DEBUG)

        testlogger.debug("subscribe and wait %s (%s)"%(topic, topic_type.__name__))
        self.assertTrue(self.subscribe_and_wait_for_message(topic, topic_type) != None)

        self.cmd_stream = rospy.Publisher('/ATRV/MotionVW', Twist)

        testlogger.debug("init sensor OK %s"%topic)

        return self.sensor_message

    def cleanup_sensor_test(self):
        self.sensor_sub.unregister()
        testlogger.debug("cleanup sensor")

    def test_sick_laser(self):
        msg = self.init_sensor_test('/ATRV/Odometry', nav_msgs.msg.Odometry)
        precision = 0.15 # we start at the origine
        self.assertAlmostEqual(msg.pose.pose.position.x, 0.0, delta=precision)
        self.assertAlmostEqual(msg.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(msg.pose.pose.position.z, 0.0, delta=precision)
        # see http://ros.org/doc/api/nav_msgs/html/msg/Odometry.html

        msg = self.init_sensor_test('/ATRV/Sick', LaserScan)

        self.assertEqual(len(msg.ranges), 181)

        # assert that : near <= distance <= far
        for distance in msg.ranges:
            self.assertTrue(distance >= 0.1 and distance <= 30)

        self.send_speed(1.0, math.pi / 2.0, 2.0)
        msg = self.wait_for_message()

        # TODO: more test

        self.cleanup_sensor_test()

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(SickLaserTest)
