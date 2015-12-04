#! /usr/bin/env python
"""
This script tests some of the base functionalities of MORSE.
"""

import sys
import math
from morse.testing.testing import MorseMoveTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def send_speed(s, morse, x, y, w, t):
    s.publish({'x' : x, 'y' : y, 'w' : w})

class Airspeed_Test(MorseMoveTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """
        robot = Morsy()

        motion = MotionXYW()
        robot.append(motion)
        motion.add_stream('socket')

        airspeed = Airspeed()
        airspeed.rotate(z = math.pi / 2) # measure speed against Y
        airspeed.add_stream('socket')
        airspeed.properties(ComputationMode = 'Velocity')
        robot.append(airspeed)
        
        airspeed_pos = Airspeed()
        airspeed_pos.rotate(z = math.pi / 2) # measure speed against Y
        airspeed_pos.add_stream('socket')
        airspeed_pos.properties(ComputationMode = 'Position')
        robot.append(airspeed_pos)


        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def _test_airspeed_helper(self, vx, vy, expected):
        delta = 0.01

        self.xyw.publish({'x' : vx, 'y': vy, 'w': 0.0})
        self.morse.sleep(0.1)

        diff_pressure = self.airspeed_stream.get()
        diff_pressure_pos = self.airspeed_pos_stream.get()

        self.assertAlmostEqual(diff_pressure['diff_pressure'], expected, delta = delta)
        self.assertAlmostEqual(diff_pressure_pos['diff_pressure'], expected, delta = delta)


    def test_airspeed(self):
        with Morse() as morse:

            self.morse = morse
            self.xyw = morse.robot.motion
            self.airspeed_stream = morse.robot.airspeed
            self.airspeed_pos_stream = morse.robot.airspeed

            self._test_airspeed_helper(0.0, 0.0, 0.0)

            # no speed in the direction of the sensor ...
            self._test_airspeed_helper(1.0, 0.0, 0.0)

            # now measuring some diff_pressure
            self._test_airspeed_helper(0.0, 1.0, 0.55)

            # scale by 4 if we increase speed by 2
            self._test_airspeed_helper(0.0, 2.0, 2.30)

            # Inversing speed, same differential_pressure
            self._test_airspeed_helper(0.0, -2.0, 2.30)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(Airspeed_Test)
