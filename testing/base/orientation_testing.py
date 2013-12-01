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
        robot.append(orientation)

        pose = Pose()
        pose.add_stream('socket')
        robot.append(pose)

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_orientation(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            gyro_stream = morse.robot.gyro
            pose_stream = morse.robot.pose

            orientation_stream = morse.robot.orientation

            precision = 0.12

            angles = gyro_stream.get()
            pose = pose_stream.get()
            self.assertAlmostEqual(angles['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['x'], 10.0, delta=precision)
            self.assertAlmostEqual(pose['y'], 8.0, delta=precision)
            self.assertAlmostEqual(pose['z'], 20.0, delta=precision)

            # Position does not change, only yaw is modified
            send_angles(orientation_stream, math.pi/2, 0.0, 0.0)
            morse.sleep(0.1)

            
            pose = pose_stream.get()
            angles = gyro_stream.get()
            self.assertAlmostEqual(angles['yaw'], math.pi/2, delta=precision)
            self.assertAlmostEqual(angles['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(angles['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['yaw'], math.pi/2, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['x'], 10.0, delta=precision)
            self.assertAlmostEqual(pose['y'], 8.0, delta=precision)
            self.assertAlmostEqual(pose['z'], 20.0, delta=precision)

            # Complete orientation settings
            send_angles(orientation_stream, -math.pi/2, math.pi/8, 0.89)
            morse.sleep(0.1)

            pose = pose_stream.get()
            angles = gyro_stream.get()
            self.assertAlmostEqual(angles['yaw'], -math.pi/2, delta=precision)
            self.assertAlmostEqual(angles['pitch'], math.pi/8, delta=precision)
            self.assertAlmostEqual(angles['roll'], 0.89, delta=precision)
            self.assertAlmostEqual(pose['yaw'], -math.pi/2, delta=precision)
            self.assertAlmostEqual(pose['pitch'], math.pi/8, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.89, delta=precision)
            self.assertAlmostEqual(pose['x'], 10.0, delta=precision)
            self.assertAlmostEqual(pose['y'], 8.0, delta=precision)
            self.assertAlmostEqual(pose['z'], 20.0, delta=precision)



########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(OrientationTest)
