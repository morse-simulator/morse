#! /usr/bin/env python
"""
This script tests the Video camera with ROS in MORSE.
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
from sensor_msgs.msg import Image

# Include this import to be able to use your test file as a regular
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class VideoCameraTest(RosTestCase):

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

        camera = VideoCamera('Camera')
        camera.properties(cam_width=128, cam_height=128, capturing=True, Vertical_Flip=True)
        camera.translate(x=0.3, z=0.76)
        camera.frequency(3)
        robot.append(camera)
        camera.add_stream('ros')

        env = Environment('indoors-1/boxes')
        # No fastmode here, no MaterialIndex in WIREFRAME mode: AttributeError:
        # 'KX_PolygonMaterial' object has no attribute 'getMaterialIndex'
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

        tmp = self.subscribe_and_wait_for_message(topic, topic_type)
        if not tmp: # remove once https://github.com/ros/genpy/pull/9 is merged
            testlogger.error("please patch ROS, see patches/ros_python3.diff")
        self.assertTrue(tmp != None)

        self.cmd_stream = rospy.Publisher('/ATRV/MotionVW', Twist)

        testlogger.debug("init sensor OK %s"%topic)

        return self.sensor_message

    def cleanup_sensor_test(self):
        self.sensor_sub.unregister()
        testlogger.debug("cleanup sensor")

    def test_video_camera(self):
        msg = self.init_sensor_test('/ATRV/Odometry', nav_msgs.msg.Odometry)
        precision = 0.15 # we start at the origine
        self.assertAlmostEqual(msg.pose.pose.position.x, 0.0, delta=precision)
        self.assertAlmostEqual(msg.pose.pose.position.y, 0.0, delta=precision)
        self.assertAlmostEqual(msg.pose.pose.position.z, 0.0, delta=precision)
        # see http://ros.org/doc/api/nav_msgs/html/msg/Odometry.html

        msg = self.init_sensor_test('/ATRV/Camera/image', Image)

        self.assertEqual(len(msg.data), 128*128*4) # RGBA

        self.send_speed(1.0, math.pi / 2.0, 2.0)
        msg = self.wait_for_message()

        # TODO: more test

        self.cleanup_sensor_test()

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(VideoCameraTest)
