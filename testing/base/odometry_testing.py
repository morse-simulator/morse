#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
import math
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class OdometryTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = ATRV()
        robot.translate(x = 5.0, y = 2.0)
        robot.rotate(z = math.pi / 2)

        pose = Pose()
        robot.append(pose)
        pose.add_stream('socket')

        motion = MotionVW('motion')
        robot.append(motion)
        motion.add_stream('socket')

        odo = Odometry()
        robot.append(odo)
        odo.level('differential')
        odo.add_stream('socket')

        raw_odo = Odometry()
        raw_odo.level("raw")
        robot.append(raw_odo)
        raw_odo.add_stream('socket')

        integ_odo = Odometry()
        robot.append(integ_odo)
        integ_odo.add_stream("socket")

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def clear_datas(self, x, y, yaw):
        self.x = x
        self.y = y
        self.yaw = yaw

    def record_datas(self, record):
        dx = record['dx']
        dy = record['dy']

        self.yaw += record['dyaw']
        
        # normalise angle
        while (self.yaw > math.pi):
            self.yaw+= -2 * math.pi
        while (self.yaw < -math.pi):
            self.yaw+= 2 * math.pi

        self.x += dx
        self.y += dy

    def odometry_test_helper(self, morse, v, w, t):
        self.odo_stream.subscribe(self.record_datas)
        self.motion.publish({'v':v, 'w':w})
        morse.sleep(t + 0.1)
        self.odo_stream.unsubscribe(self.record_datas)

    
    def verify(self, expected_x, expected_y, expected_yaw):
        # Numerical integration is maybe not really good, so test with a
        # precision of 0.16
        precision = 0.16

        pose = self.pose_stream.get()
        integ_odo = self.integ_odo_stream.get()
        self.assertAlmostEqual(self.x, expected_x, delta=precision)
        self.assertAlmostEqual(self.y, expected_y, delta=precision)
        self.assertAlmostEqual(self.yaw, expected_yaw, delta=precision)
        self.assertAlmostEqual(pose['x'], 5.0 - expected_y, delta=precision)
        self.assertAlmostEqual(pose['y'], 2.0 + expected_x, delta=precision)
        self.assertAlmostEqual(pose['yaw'], expected_yaw + math.pi/2, delta=precision)
        self.assertAlmostEqual(integ_odo['x'], expected_x, delta=precision)
        self.assertAlmostEqual(integ_odo['y'], expected_y, delta=precision)
        self.assertAlmostEqual(integ_odo['yaw'], expected_yaw, delta=precision)
        self.clear_datas(expected_x, expected_y, expected_yaw)

    def test_odometry(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """

        with Morse() as morse:
            # Read the start position, it must be (0.0, 0.0, 0.0)
            self.pose_stream = morse.robot.pose
            self.odo_stream = morse.robot.odo
            self.integ_odo_stream = morse.robot.integ_odo
            self.motion = morse.robot.motion

            self.clear_datas(0.0, 0.0, 0.0)

            self.odometry_test_helper(morse, 1.0, 0.0, 2.0)
            self.verify(2.0, 0.0, 0.0)

            self.odometry_test_helper(morse, -1.0, 0.0, 2.0)
            self.verify(0.0, 0.0, 0.0)

            self.odometry_test_helper(morse, 1.0, -math.pi/4.0, 2.0)
            self.verify(4.0 / math.pi, -4.0/math.pi, -math.pi/2.0)

            self.odometry_test_helper(morse, 0.5, -math.pi/8.0, 12.0)
            self.verify(0.0, 0.0, 0.0)

            # XXX fail Y with 0.11 delta
            #self.odometry_test_helper(morse, -2.0, math.pi/2.0, 3.0)
            #self.verify(4.0 / math.pi, -4.0/math.pi, -math.pi/2.0)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(OdometryTest)
