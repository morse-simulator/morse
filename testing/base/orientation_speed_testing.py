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
import math
from pymorse import Morse

def send_angles(s, yaw, pitch, roll):
    s.publish({'yaw' : yaw, 'pitch' : pitch, 'roll' : roll})

class OrientationTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = RMax('robot')
        robot.translate(10.0, 8.0, 20.0)
        
        gyro = Gyroscope()
        gyro.add_stream('socket')
        robot.append(gyro)

        orientation = Orientation('orientation')
        orientation.add_stream('socket')
        orientation.properties(speed = 0.5)
        robot.append(orientation)

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_orientation(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            gyro_stream = morse.robot.gyro
            orientation_stream = morse.robot.orientation

            precision = 0.05

            angles = gyro_stream.get()
            self.assertAlmostEqual(angles['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['roll'], 0.0, delta=precision)

            send_angles(orientation_stream, 3.0, 0.0, 0.0)
            morse.sleep(1.0)
            angles = gyro_stream.get()

            self.assertAlmostEqual(angles['yaw'], 0.5, delta = precision)
            self.assertAlmostEqual(angles['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['roll'], 0.0, delta=precision)

            morse.sleep(6.0)
            angles = gyro_stream.get()

            self.assertAlmostEqual(angles['yaw'], 3.0, delta = precision)
            self.assertAlmostEqual(angles['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['roll'], 0.0, delta=precision)

            # Be smart on the rotation order
            send_angles(orientation_stream, -3.0, 0.0, 0.0)
            morse.sleep(1.0)

            angles = gyro_stream.get()
            self.assertAlmostEqual(angles['yaw'], -3.0, delta = precision)
            self.assertAlmostEqual(angles['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['roll'], 0.0, delta=precision)

            # Turn in the order direction
            send_angles(orientation_stream, 2.0, 0.0, 0.0)
            morse.sleep(1.0)

            angles = gyro_stream.get()
            self.assertAlmostEqual(angles['yaw'], 2.8, delta = precision)
            self.assertAlmostEqual(angles['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['roll'], 0.0, delta=precision)

            morse.sleep(2.0)
            angles = gyro_stream.get()
            self.assertAlmostEqual(angles['yaw'], 2.0, delta = precision)
            self.assertAlmostEqual(angles['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['roll'], 0.0, delta=precision)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(OrientationTest)
