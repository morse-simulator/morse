#! /usr/bin/env python
"""
This script tests the Video camera with ROS in MORSE.
"""

import sys
import time
import math
import struct
from morse.testing.ros import RosTestCase
from morse.testing.testing import testlogger

import roslib
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image

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

class VideoCameraRosTest(RosTestCase):

    def setUpEnv(self):
        """ Defines the test scenario """

        robot = ATRV()

        motion = MotionVW()
        robot.append(motion)
        motion.add_stream('ros')

        camera = VideoCamera()
        camera.properties(cam_width=128, cam_height=128, capturing=True, Vertical_Flip=True)
        camera.translate(z = 1)
        camera.frequency(3)
        robot.append(camera)
        camera.add_stream('ros')

        env = Environment('indoors-1/boxes')
        # No fastmode here, no MaterialIndex in WIREFRAME mode: AttributeError:
        # 'KX_PolygonMaterial' object has no attribute 'getMaterialIndex'

    def test_video_camera(self):
        rospy.init_node('morse_ros_video_testing', log_level=rospy.DEBUG)

        motion_topic = '/robot/motion'
        camera_topic = '/robot/camera/image'

        pub_vw(motion_topic, 1, 1)

        old = []
        for step in range(2):
            msg = rospy.wait_for_message(camera_topic, Image, 10)

            self.assertEqual(len(msg.data), 128*128*4) # RGBA
            self.assertTrue (msg.data != old)
            old = msg.data

            time.sleep(0.2) # wait for turning

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(VideoCameraRosTest, time_modes = [TimeStrategies.BestEffort])
