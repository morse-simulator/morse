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

class Waypoints_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        
        robot = Robot('atrv')

        pose = Sensor('pose')
        pose.translate(z=-0.10) # atrv body
        robot.append(pose)
        pose.configure_mw('socket')

        motion = Actuator('waypoint')
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
            for coord in pose.values():
                self.assertAlmostEqual(coord, 0.0, delta=0.02)

            # waypoint controller socket
            port = morse.get_stream_port('Motion_Controller')
            v_w_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            v_w_client.connect(('localhost', port))

            v_w_client.send(json.dumps({'x' : 10.0, 'y': 5.0, 'z': 0.0, 
                                         'tolerance' : 0.5, 
                                         'speed' : 1.0}).encode());
            sleep(15)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 10.0, delta=0.5)
            self.assertAlmostEqual(pose['y'], 5.0, delta=0.5)


            # test tolerance parameter
            v_w_client.send(json.dumps({'x' : 0.0, 'y': 0.0, 'z': 0.0, 
                                         'tolerance' : 2.5, 
                                         'speed' : 1.0}).encode());
            sleep(15)
            pose = pose_stream.get()
            distance_goal = math.sqrt( pose['x'] * pose['x'] + pose['y'] * pose['y'])
            self.assertLess(distance_goal, 2.5)
            self.assertGreater(distance_goal, 2.0)

    def test_waypoint_service_controller(self):
        with Morse() as morse:
            # Read the start position, it must be (0.0, 0.0, 0.0)
            pose_stream = morse.stream('Pose')
            pose = pose_stream.get()
            for coord in pose.values():
                self.assertAlmostEqual(coord, 0.0, delta=0.02)

            morse.call_server('Motion_Controller', 'goto', 10.0, 5.0, 0.0, 0.5, 1.0)

            pose = pose_stream.get()
            self.assertAlmostEqual(pose['x'], 10.0, delta=0.5)
            self.assertAlmostEqual(pose['y'], 5.0, delta=0.5)

            # XXX need to test other services offered by waypoint
            # controller, but pymorse support is not good enough at the
            # moment


########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(Waypoints_Test)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

