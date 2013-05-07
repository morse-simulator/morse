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

import sys
from pymorse import Morse


class UTMModifierTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = ATRV()
        robot.translate(10.0, 8.0, 0.0)
        
        gps = GPS()
        gps.add_stream('socket')
        robot.append(gps)

        gps_mod = GPS()
        gps_mod.add_stream('socket')
        gps_mod.alter('UTM', z_offset=421)
        robot.append(gps_mod)

        env = Environment('empty', fastmode = True)
        env.add_service('socket')
        # Need to put float greater than 10000 in string
        env.properties(UTMXOffset='123456789.0', UTMYOffset=-4242.0)


    def test_read_gps(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            gps_stream = morse.robot.gps
            gps_mod_stream = morse.robot.gps_mod

            pos = gps_stream.get()
            pos_mod = gps_mod_stream.get()

            precision = 0.02
            self.assertAlmostEqual(pos['x'], 10.0, delta=precision)
            self.assertAlmostEqual(pos['y'], 8.0, delta=precision)
            # Z = 0.1 : pose of the ATRV's center relative to the world
            self.assertAlmostEqual(pos['z'], 0.1, delta=precision)

            self.assertAlmostEqual(pos_mod['x'], 10.0 + 123456789.0, delta=precision)
            self.assertAlmostEqual(pos_mod['y'], 8.0 + -4242.0, delta=precision)
            self.assertAlmostEqual(pos_mod['z'], 421.1, delta=precision)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(UTMModifierTest)
