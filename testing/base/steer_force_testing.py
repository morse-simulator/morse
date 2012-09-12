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

def send_force(s, steer, force, brake):
    s.send(json.dumps({'steer' : steer, 'force' : force, 'brake' : brake}).encode())


class SteerForceTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = Robot('hummer')
        
        pose = Sensor('pose')
        pose.configure_mw('socket')
        robot.append(pose)

        steer_force = Actuator('steer_force')
        robot.append(steer_force)
        steer_force.configure_mw('socket')

        env = Environment('land-1/rosace_1')
        env.configure_service('socket')

    def test(self):
        with Morse() as morse:
            pose_stream = morse.stream('Pose')

            pose = pose_stream.get()
            # It is not really precise to control the robot in this way
            # in open loop. So it is a highly qualitative test.

            x = pose['x']
            y = pose['y']

            # destination socket
            port = morse.get_stream_port('Motion_Controller')
            steer_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            steer_client.connect(('localhost', port))

            send_force(steer_client, 0.0, -20.0, 0.0)
            time.sleep(3.0)
            send_force(steer_client, 0.0, 0.0, 10.0)
            time.sleep(1.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], x + 8.0, delta = 1.0)
            self.assertAlmostEqual(pose['y'], y, delta = 1.0)



            # Doubling the force 
            send_force(steer_client, 0.0, -40.0, 0.0)
            time.sleep(3.0)
            send_force(steer_client, 0.0, 0.0, 10.0)
            time.sleep(2.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], x + 26.5, delta = 1.0)
            self.assertAlmostEqual(pose['y'], y, delta = 1.0)

            # Backward move
            send_force(steer_client, 0.0, 10.0, 0.0)
            time.sleep(10.5)
            send_force(steer_client, 0.0, 0.0, 10.0)
            time.sleep(2.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], x, delta = 1.0)
            self.assertAlmostEqual(pose['y'], y, delta = 1.0)

            # Turning
            send_force(steer_client, -1.0, -10.0, 0.0)
            time.sleep(10)
            send_force(steer_client, 0.0, 0.0, 10.0)
            time.sleep(2.0)
            pose = pose_stream.get()

            self.assertAlmostEqual(pose['yaw'], 2.25, delta = 0.2)
            self.assertAlmostEqual(pose['x'], x - 2.5, delta = 1.0)
            self.assertAlmostEqual(pose['y'], x - 6.0, delta = 1.0)



########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(SteerForceTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

