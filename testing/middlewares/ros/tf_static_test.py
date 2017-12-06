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
from morse.middleware.ros.tfMessage import tfMessage

from time import sleep

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class StaticTfTest(RosTestCase):

    def setUpEnv(self):
        
        robot = Morsy('morsy_test_1')
        pose = Pose()
        robot.append(pose)

        pose.translate(0, 1, 2)
        pose.add_stream('ros',
            'morse.middleware.ros.static_tf.StaticTFPublisher')
        pose.add_stream('ros',
            'morse.middleware.ros.static_tf.StaticTF2Publisher')

        env = Environment('empty', fastmode = True)

        
    def _callback(self, m, precision = 0.01):
        t = m.transforms[0]
        if t.child_frame_id != '/morsy_test_1/pose':
            return
        
        p = t.transform.translation
        self.assertAlmostEqual(p.x, 0.0, delta=precision)
        self.assertAlmostEqual(p.y, 1.0, delta=precision)
        self.assertAlmostEqual(p.z, 2.0, delta=precision)

        m_time = t.header.stamp.to_sec()
        if self.m_time is not None:
            self.assertTrue(m_time > self.m_time + 1.0)

        self.m_time = m_time
        self.n_received += 1
        

    def test_static_tf(self):

        rospy.init_node('morse_ros_static_tf_test')
        
        self.m_time = None
        self.n_received = 0
          
        self.tf = rospy.Subscriber("/tf", tfMessage, self._callback,
                                   queue_size = 1)

        sleep(2.5)

        self.assertTrue(self.n_received >= 2)


    def test_static_tf2(self):
        rospy.init_node('morse_ros_static_tf_test')

        self.m_time = None
        self.n_received = 0
          
        self.tf = rospy.Subscriber("/tf_static", tfMessage, self._callback,
                                   queue_size = 1)

        sleep(0.5)
        
        self.assertTrue(self.n_received == 1)

        
########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(StaticTfTest, time_modes = [TimeStrategies.BestEffort])
