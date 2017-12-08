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

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo

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
        rospy.init_node('morse_ros_video_testing')

        motion_topic = '/robot/motion'
        camera_topic = '/robot/camera/image'
        camnfo_topic = '/robot/camera/camera_info'

        pub_vw(motion_topic, 1, 1)

        old = []
        for step in range(2):
            msg = rospy.wait_for_message(camnfo_topic, CameraInfo, 10)
            camera_info_frame = msg.header.frame_id
            # might want to add more CameraInfo test here

            msg = rospy.wait_for_message(camera_topic, Image, 10)

            self.assertEqual(msg.header.frame_id, camera_info_frame)

            self.assertEqual(len(msg.data), 128*128*4) # RGBA
            # dont use assertNotEqual here
            #   dumping raw image data in log is not relevant
            self.assertTrue(msg.data != old)
            old = msg.data

            time.sleep(0.2) # wait for turning

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(VideoCameraRosTest, time_modes = [TimeStrategies.BestEffort])
