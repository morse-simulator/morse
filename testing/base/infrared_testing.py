#! /usr/bin/env python
"""
This script tests the Infrared range sensor in MORSE.
"""

import sys
import math
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class InfraredTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario. """
        robot = ATRV()
        robot.rotate(z = math.pi)
        robot.translate(x = -4.5)

        infrared = Infrared()
        infrared.translate(x=0.9)
        robot.append(infrared)
        infrared.add_stream('socket')

        env = Environment('indoors-1/boxes', fastmode = True)
        env.add_service('socket')

    def test_infrared(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
            data = morse.robot.infrared.get()
            assert(20 < len(data['range_list']) < 22)
            assert(not [r for r in data['range_list'] if not 0 < r < 5])
            # see depth_camera_testing for further tests.

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(InfraredTest)
