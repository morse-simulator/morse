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

class LevelsTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = ATRV()

        odo = Odometry() # odometry level default to 'differential'
        robot.append(odo)
        odo.add_stream('socket')

        diff_odo = Odometry()
        diff_odo.level("differential")
        robot.append(diff_odo)
        diff_odo.add_stream('socket')


        raw_odo = Odometry()
        raw_odo.level("raw")
        robot.append(raw_odo)
        raw_odo.add_stream('socket')

        integ_odo = Odometry()
        integ_odo.level("integrated")
        robot.append(integ_odo)
        integ_odo.add_stream("socket")

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_levels(self):

        with Morse() as morse:

            odo = morse.robot.odo.get()
            diff_odo = morse.robot.diff_odo.get()
            raw_odo = morse.robot.raw_odo.get()
            integ_odo = morse.robot.integ_odo.get()

            self.assertEquals(set(['timestamp', 'dS']), set(raw_odo.keys()))
            self.assertEquals(set(['timestamp',
                                   'x', 'y', 'z', 'yaw', 'pitch', 'roll',
                                   'vx', 'vy', 'vz', 'wz', 'wy', 'wx']),
                              set(integ_odo.keys()))
            self.assertEquals(set(['timestamp', 'dx', 'dy', 'dz', 'dyaw', 'dpitch', 'droll']),
                              set(diff_odo.keys()))
            self.assertEquals(set(integ_odo.keys()), set(odo.keys()))


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(LevelsTest)
