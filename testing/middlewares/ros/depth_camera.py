#! /usr/bin/env python
"""
This script tests the Depth camera with ROS in MORSE.
"""

import sys
import time
import math
import struct
from morse.testing.ros import RosTestCase
from morse.testing.testing import testlogger

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2

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

class DepthCameraRosTest(RosTestCase):

    def setUpEnv(self):
        """ Defines the test scenario """

        robot = ATRV()

        motion = MotionVW()
        robot.append(motion)
        motion.add_stream('ros')

        camera = DepthCamera()
        camera.translate(z = 1)
        camera.frequency(3)
        robot.append(camera)
        camera.add_stream('ros')

        env = Environment('indoors-1/boxes')
        # No fastmode here, no MaterialIndex in WIREFRAME mode: AttributeError:
        # 'KX_PolygonMaterial' object has no attribute 'getMaterialIndex'

    def test_depth_camera(self):
        rospy.init_node('morse_ros_depth_testing')

        motion_topic = '/robot/motion'
        camera_topic = '/robot/camera'

        pub_vw(motion_topic, 1, 1)

        for step in range(5):
            msg = rospy.wait_for_message(camera_topic, PointCloud2, 10)

            # assert that : near <= z <= far
            for i in range(0, len(msg.data) - 12, 12):
                xyz = struct.unpack('fff', msg.data[i:i+12])
                self.assertGreaterEqual(xyz[2], 1)
                self.assertLessEqual(xyz[2], 20)

            time.sleep(0.2) # wait for turning

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(DepthCameraRosTest, time_modes = [TimeStrategies.BestEffort])
