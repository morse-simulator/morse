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

def send_goal(s, x, y, z):
    s.send(json.dumps({'x' : x, 'y' : y, 'z' : z}).encode())

class DestinationTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = Robot('rmax')
        robot.translate(0.0, 0.0, 20.0)
        
        pose = Sensor('pose')
        pose.configure_mw('socket')
        robot.append(pose)

        destination = Actuator('destination')
        robot.append(destination)
        destination.configure_mw('socket')
        destination.properties(Speed=2.0, Tolerance=0.3)

        env = Environment('indoors-1/indoor-1')
        env.configure_service('socket')

    def test(self):
        with Morse() as morse:
            pose_stream = morse.stream('Pose')

            pose = pose_stream.get()
            precision = 0.02
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['x'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['y'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['z'], 20.0, delta=0.1)

            # destination socket
            port = morse.get_stream_port('Motion_Controller')
            dest_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            dest_client.connect(('localhost', port))

            send_goal(dest_client, 10.0, 0.0, 20.0)

            time.sleep(3.0)
            # Only x has changed. Check that speed is respected

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['x'], 3.0 * 2.0, delta=0.1)
            self.assertAlmostEqual(pose['y'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['z'], 20.0, delta=0.1)

            time.sleep(2.0)
            # Only x has changed
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['x'], 9.7, delta=0.1)
            self.assertAlmostEqual(pose['y'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['z'], 20.0, delta=0.1)

            x = pose['x']
            send_goal(dest_client, x, 10.0, 20.0)
            time.sleep(5.0)

            # Only Y has changed
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['x'], x, delta=0.1)
            self.assertAlmostEqual(pose['y'], 9.7, delta=0.1)
            self.assertAlmostEqual(pose['z'], 20.0, delta=0.1)


            x = pose['x']
            y = pose['y']
            z = pose['z']

            send_goal(dest_client, x, y, 30.0)
            time.sleep(5.0)

            # Only Z has changed
            # XXX precision is not really good
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['x'], x, delta=precision)
            self.assertAlmostEqual(pose['y'], y, delta=precision)
            self.assertAlmostEqual(pose['z'], 30.0, delta=0.2)

            send_goal(dest_client, 0, 0, 20)
            time.sleep(10.0)
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(pose['roll'], 0.0, delta=precision)
            self.assertTrue(math.fabs(pose['x']) < 0.3)
            self.assertTrue(math.fabs(pose['y']) < 0.3)
            self.assertTrue(math.fabs(pose['z'] - 20) < 0.3)


########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(DestinationTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

