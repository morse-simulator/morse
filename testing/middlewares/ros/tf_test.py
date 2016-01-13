#! /usr/bin/env python
"""
This script tests that MORSE can generate valid tf frames.
"""

import sys
import math
from morse.testing.ros import RosTestCase
from morse.testing.testing import testlogger

from pymorse import Morse

import rospy
from tf import TransformListener

from time import sleep

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class TfTest(RosTestCase):

    def setUpEnv(self):
        
        robot = ATRV()

        odometry = Odometry()
        robot.append(odometry)
        odometry.add_stream('ros')

        motion = Teleport()
        robot.append(motion)
        motion.add_stream('socket')
        
        robot2 = ATRV()
        robot2.translate(0,1,0)
        odometry2 = Odometry()
        robot2.append(odometry2)
        odometry2.add_stream('ros', frame_id="map", child_frame_id="robot2")

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def _check_pose(self, frame1, frame2, position, quaternion, precision = 0.01):

        t = self.tf.getLatestCommonTime(frame1, frame2)
        observed_position, observed_quaternion = self.tf.lookupTransform(frame1, frame2, t)

        for a,b in zip(position, observed_position):
            self.assertAlmostEqual(a, b, delta = precision)
        for a,b in zip(quaternion, observed_quaternion):
            self.assertAlmostEqual(a, b, delta = precision)



    def test_tf(self):

        rospy.init_node('morse_ros_tf_test')

        self.tf = TransformListener()

        sleep(1)
        self.assertTrue(self.tf.frameExists("/odom"))
        self.assertTrue(self.tf.frameExists("/base_footprint"))
        self.assertTrue(self.tf.frameExists("/map"))
        self.assertTrue(self.tf.frameExists("/robot2"))

        self._check_pose("odom", "base_footprint", [0,0,0], [0,0,0,1])
        self._check_pose("map", "robot2", [0,0,0], [0,0,0,1])

        with Morse() as morse:
            teleport = morse.robot.motion

            teleport.publish({'x' : 2, 'y' : 0, 'z' : 0, \
                              'yaw' : 0, 'pitch' : 0.0, 'roll' : 0.0})
            morse.sleep(0.1)

        self._check_pose("odom", "base_footprint", [2,0,-0.1], [0,0,0,1])

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(TfTest, time_modes = [TimeStrategies.BestEffort])
