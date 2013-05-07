#! /usr/bin/env python
"""
This script tests the 'data stream' oriented feature of the socket interface.
"""

from morse.testing.testing import MorseTestCase

try:
    # Include this import to be able to use your test file as a regular 
    # builder script, ie, usable with: 'morse [run|exec] <your test>.py
    from morse.builder import *
except ImportError:
    pass

import sys
from pymorse import Morse


class GyroTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = RMax('robot')
        robot.translate(0.0, 0.0, 40.0)
        robot.rotate(math.pi/16, math.pi/8, math.pi/2)
        
        gyro = Gyroscope()
        gyro.add_stream('socket')
        robot.append(gyro)

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_read_gyro(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            gyro_stream = morse.robot.gyro

            angles = gyro_stream.get()
            precision = 0.02
            self.assertAlmostEqual(angles['yaw'], math.pi/2, delta=precision)
            self.assertAlmostEqual(angles['pitch'], math.pi/8, delta=precision)
            self.assertAlmostEqual(angles['roll'], math.pi/16, delta=precision)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(GyroTest)
