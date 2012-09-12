#! /usr/bin/env python
"""
This script tests the SICK laser range sensor in MORSE.
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

class Semantic_Camera_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = Robot('atrv')

        camera = Sensor('semantic_camera')
        robot.append(camera)
        camera.translate(x=0.2000, z=0.9000)
        camera.configure_mw('socket')

        motion = Actuator('teleport')
        robot.append(motion)
        motion.configure_mw('socket')

        #env = Environment('indoors-1/indoor-1')
        env = Environment('indoors-1/boxes')
        env.configure_service('socket')


    def test_semantic_camera(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
            # Read the data from the semantic camera
            semantic_stream = morse.stream('CameraMain')

            port = morse.get_stream_port('Motion_Controller')
            teleport_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            teleport_client.connect(('localhost', port))

            objects = semantic_stream.get()
            self.assertEqual(objects, [])

            # Change the orientation of the robot using the v_w socket
            send_dest(teleport_client, 0.0, 0.0, 5.0/4.0 * math.pi)

            # Second test for the sensor, with objects in front
            objects = semantic_stream.get()
            self.assertEqual(objects[0]['name'],'BlueBox')
            self.assertAlmostEqual(objects[0]['position']['x'], -3.48, delta=0.1)
            self.assertAlmostEqual(objects[0]['position']['y'], -3.0, delta=0.1)

            send_dest(teleport_client, -5.0, 0.0, math.pi)

            objects = semantic_stream.get()
            self.assertEqual(objects[0]['name'],'RedBox')
            self.assertAlmostEqual(objects[0]['position']['x'], -7.48, delta=0.1)
            self.assertAlmostEqual(objects[0]['position']['y'], 0.0, delta=0.1)

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(Semantic_Camera_Test)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())
