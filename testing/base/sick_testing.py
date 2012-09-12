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

def send_speed(s, v, w, t):
    s.send(json.dumps({'v' : v, 'w' : w}).encode())
    sleep(t)
    s.send(json.dumps({'v' : 0.0, 'w' : 0.0}).encode())

class Sick_Test(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = Robot('atrv')

        sick = Sensor('sick')
        sick.translate(z=0.9)
        sick.properties(laser_range = 10.0)
        robot.append(sick)
        sick.configure_mw('socket')

        motion = Actuator('v_omega')
        robot.append(motion)
        motion.configure_mw('socket')

        #env = Environment('indoors-1/indoor-1')
        env = Environment('indoors-1/boxes')
        env.configure_service('socket')


    def test_sick(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
        
            # Read the data from the sick sensor
            self.sick_stream = morse.stream('Sick')

            port = morse.get_stream_port('Motion_Controller')
            self.v_w_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.v_w_client.connect(('localhost', port))

            sleep(5)
            # Initial test of the sensor with no visible objects
            sick = self.sick_stream.get()
            for ray in sick['point_list']:
                self.assertAlmostEqual(ray[0], 0.0, delta=0.05)
                self.assertAlmostEqual(ray[1], 0.0, delta=0.05)
                self.assertAlmostEqual(ray[2], 0.0, delta=0.05)
            for length in sick['range_list']:
                self.assertAlmostEqual(length, 10.0, delta=0.05)
     
            # Change the orientation of the robot using the v_w socket
            send_speed(self.v_w_client, 0.0, math.pi/2.0, 2.0)

            # Second test for the sensor, with objects in front
            sick = self.sick_stream.get()

            # First few rays should not hit anything
            for index in range(30):
                ray = sick['point_list'][index]
                self.assertAlmostEqual(ray[0], 0.0, delta=0.05)
                self.assertAlmostEqual(ray[1], 0.0, delta=0.05)
                self.assertAlmostEqual(ray[2], 0.0, delta=0.05)
                length = sick['range_list'][index]
                self.assertAlmostEqual(length, 10.0, delta=0.05)

            # Make particular tests for a few rays at locations
            #  known to have objects
            ray = sick['point_list'][45]
            self.assertAlmostEqual(ray[0], 6.0, delta=0.05)
            self.assertAlmostEqual(ray[1], -6.0, delta=0.05)
            length = sick['range_list'][45]
            self.assertAlmostEqual(length, 8.485, delta=0.05)

            ray = sick['point_list'][90]
            self.assertAlmostEqual(ray[0], 7.0, delta=0.05)
            self.assertAlmostEqual(ray[1], 0.0, delta=0.05)
            length = sick['range_list'][90]
            self.assertAlmostEqual(length, 7.0, delta=0.05)

            ray = sick['point_list'][135]
            self.assertAlmostEqual(ray[0], 3.0, delta=0.05)
            self.assertAlmostEqual(ray[1], 3.0, delta=0.05)
            length = sick['range_list'][135]
            self.assertAlmostEqual(length, 4.243, delta=0.05)

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(Sick_Test)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

