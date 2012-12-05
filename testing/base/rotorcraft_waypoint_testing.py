#! /usr/bin/env python
"""
This script tests the waypoints actuator, both the data and service api
"""

import sys
import socket
import json
import math
from time import sleep
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class RotorcraftWaypoints_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = Robot('quadrotor_dynamic')
        robot.translate(x = -1.24, y=1.70, z=1.81)

        pose = Sensor('pose')
        robot.append(pose)
        pose.configure_mw('socket')

        motion = Actuator('rotorcraft_waypoint')
        robot.append(motion)
        motion.configure_mw('socket')
        motion.configure_service('socket')

        
        env = Environment('indoors-1/indoor-1')
        env.configure_service('socket')

    def test_waypoint_controller(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
        
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.stream('Pose')
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], -1.24, delta=0.05)
            self.assertAlmostEqual(pose['y'], 1.70, delta=0.05)
            self.assertAlmostEqual(pose['z'], 1.81, delta=0.05)
            self.assertAlmostEqual(pose['yaw'], 0, delta=0.05)


            # waypoint controller socket
            port = morse.get_stream_port('Motion_Controller')
            wp_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            wp_client.connect(('localhost', port))

            wp_client.send(json.dumps({'x' : 10.0, 'y': 5.0, 'z': 10.0, 
                                         'tolerance' : 0.5, 
                                         'yaw' : 1.0}).encode());
            sleep(10)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 10.0, delta=0.5)
            self.assertAlmostEqual(pose['y'], 5.0, delta=0.5)
            self.assertAlmostEqual(pose['z'], 10.0, delta=0.5)
            self.assertAlmostEqual(pose['yaw'], 1.0, delta=0.05)


    def test_waypoint_service_controller(self):
        with Morse() as morse:
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.stream('Pose')
            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], -1.24, delta=0.05)
            self.assertAlmostEqual(pose['y'], 1.70, delta=0.05)
            self.assertAlmostEqual(pose['z'], 1.81, delta=0.05)
            self.assertAlmostEqual(pose['yaw'], 0, delta=0.05)

            morse.call_server('Motion_Controller', 'goto', 10.0, 5.0, 10.0, 1.0, 0.5)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 10.0, delta=0.5)
            self.assertAlmostEqual(pose['y'], 5.0, delta=0.5)
            self.assertAlmostEqual(pose['z'], 10.0, delta=0.5)
            self.assertAlmostEqual(pose['yaw'], 1.0, delta=0.05)



########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(RotorcraftWaypoints_Test)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

