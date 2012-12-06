#! /usr/bin/env python
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
import json
import time
from pymorse import Morse

def send_dest(s, x, y, yaw):
    s.send(json.dumps({'x' : x, 'y' : y, 'z' : 0, 'yaw' : yaw, 'pitch' : 0.0, 'roll' : 0.0}).encode())
    time.sleep(0.1)

class ThermometerTest(MorseTestCase):

    def setUpEnv(self):
        robot = Robot('atrv')
        robot.translate(z=0.2)

        thermometer = Sensor('thermometer')
        robot.append(thermometer)
        thermometer.configure_mw('socket')

        motion = Actuator('teleport')
        robot.append(motion)
        motion.configure_mw('socket')

        env = Environment('land-1/rosace_1')
        env.properties(Temperature='25.0')
        env.configure_service('socket')

    def test_temperature(self):
        with Morse() as morse:
            temp_stream = morse.stream('Thermometer')

            o = temp_stream.get()
            # destination socket
            self.assertAlmostEqual(o['temperature'], 25.0, delta=0.01)

            port = morse.get_stream_port('Motion_Controller')
            teleport_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            teleport_client.connect(('localhost', port))

            send_dest(teleport_client, 40.0, 0.0, 0.0)
            time.sleep(0.1)

            # We are nearer of the fire so the temperature is expected
            # to be hotter.
            o = temp_stream.get()
            temp = o['temperature']
            self.assertGreater(temp, 28.0)

            # It must be hotter and hotter ... 
            time.sleep(10.0)
            o = temp_stream.get()
            self.assertGreater(o['temperature'], temp)





########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(ThermometerTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

