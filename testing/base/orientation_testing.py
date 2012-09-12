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

import os
import sys
import socket
import math
import json
import time
from pymorse import Morse

def send_angles(s, yaw, pitch, roll):
    s.send(json.dumps({'yaw' : yaw, 'pitch' : pitch, 'roll' : roll}).encode())

class OrientationTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = Robot('rmax')
        robot.translate(10.0, 8.0, 20.0)
        
        gyro = Sensor('gyroscope')
        gyro.configure_mw('socket')
        robot.append(gyro)

        orientation = Actuator('orientation')
        orientation.configure_mw('socket')
        robot.append(orientation)

        pose = Sensor('pose')
        pose.configure_mw('socket')
        robot.append(pose)

        env = Environment('indoors-1/indoor-1')
        env.configure_service('socket')

    def test_orientation(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            gyro_stream = morse.stream('Gyroscope')
            pose_stream = morse.stream('Pose')

            port = morse.get_stream_port('Motion_Controller')
            orientation_stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            orientation_stream.connect(('localhost', port))

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
            time.sleep(0.1)

            
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
            time.sleep(0.1)

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
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(OrientationTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

