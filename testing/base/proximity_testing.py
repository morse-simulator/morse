#! /usr/bin/env python
"""
This script tests the proximity sensor in MORSE.
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

def send_dest(s, x, y, yaw):
    s.send(json.dumps({'x' : x, 'y' : y, 'z' : 0, 'yaw' : yaw, 'pitch' : 0.0, 'roll' : 0.0}).encode())
    sleep(0.1)

class ProximityTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = Robot('atrv')

        proximity = Sensor('proximity')
        proximity.translate(z=0.5)
        proximity.properties(Track = "Catch_me")
        proximity.properties(Range = 2.0)
        robot.append(proximity)
        proximity.configure_mw('socket')
        proximity.configure_service('socket')

        pose = Sensor('pose')
        robot.append(pose)
        pose.configure_mw('socket')

        motion = Actuator('teleport')
        robot.append(motion)
        motion.configure_mw('socket')

        target1 = Robot('atrv')
        target1.name = "Target1"
        target1.properties(Catch_me = True)
        target1.translate(x=10.0, y = 1.0)

        target2 = Robot('atrv')
        target2.name = "Target2"
        target2.properties(Catch_me2 = True)
        target2.translate(x=10.0, y = -1.0)

        target3 = Robot('atrv')
        target3.name = "Target3"
        target3.properties(Catch_me = True)
        target3.translate(x=-4.0, y = 0.0)

        env = Environment('indoors-1/indoor-1')
        env.configure_service('socket')

    def test_proximity(self):
        with Morse() as morse:
        
            prox_stream = morse.stream('Proximity')

            port = morse.get_stream_port('Motion_Controller')
            teleport_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            teleport_client.connect(('localhost', port))

            prox = prox_stream.get()
            self.assertEqual(len(prox['near_objects']), 0)

            # still emtpy
            send_dest(teleport_client, 8.0, 0.0, 0.0)
            prox = prox_stream.get()
            self.assertEqual(len(prox['near_objects']), 0)

            # one more meter, must find target1. target2 is at equal
            # distance but don't have the good tag
            send_dest(teleport_client, 9.0, 0.0, 0.0)
            prox = prox_stream.get()
            self.assertEqual(len(prox['near_objects']), 1)
            self.assertTrue('Target1' in prox['near_objects'])

            # Don't care about the direction, only check the distance
            send_dest(teleport_client, -2.0, 0.0, 0.0)
            prox = prox_stream.get()
            self.assertEqual(len(prox['near_objects']), 1)
            self.assertTrue('Target3' in prox['near_objects'])

            # Call the set_range service and check if we can catch the
            # two objects
            morse.call_server('Proximity', 'set_range', 20.0)
            prox = prox_stream.get()
            self.assertEqual(len(prox['near_objects']), 2)
            self.assertTrue('Target1' in prox['near_objects'])
            self.assertTrue('Target3' in prox['near_objects'])

            # Call the set_tracked_tag service and check if we catch
            # target2
            morse.call_server('Proximity', 'set_tracked_tag', 'Catch_me2')
            prox = prox_stream.get()
            self.assertEqual(len(prox['near_objects']), 1)
            self.assertTrue('Target2' in prox['near_objects'])


########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(ProximityTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

