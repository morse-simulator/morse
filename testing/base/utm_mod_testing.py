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
import json
from pymorse import Morse


class UTMModifierTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = Robot('atrv')
        robot.translate(10.0, 8.0, 0.0)
        
        gps = Sensor('gps')
        gps.configure_mw('socket')
        robot.append(gps)

        gps_mod = Sensor('gps')
        gps_mod.configure_mw('socket')
        gps_mod.configure_modifier('UTM')
        robot.append(gps_mod)

        env = Environment('indoors-1/indoor-1')
        env.configure_service('socket')
        env.properties(UTMXOffset=1337.0, UTMYOffset=-4242.0, UTMZOffset=421.0)


    def test_read_gps(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            gps_stream = morse.stream('GPS')
            gps_mod_stream = morse.stream('GPS.001')

            pos = gps_stream.get()
            pos_mod = gps_mod_stream.get()

            precision = 0.02
            self.assertAlmostEqual(pos['x'], 10.0, delta=precision)
            self.assertAlmostEqual(pos['y'], 8.0, delta=precision)
            self.assertAlmostEqual(pos['z'], 0.0, delta=precision)

            self.assertAlmostEqual(pos_mod['x'], 10.0 + 1337.0, delta=precision)
            self.assertAlmostEqual(pos_mod['y'], 8.0 + -4242.0, delta=precision)
            self.assertAlmostEqual(pos_mod['z'], 421.0, delta=precision)

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(UTMModifierTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

