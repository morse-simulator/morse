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


class MagnemoterTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = Morsy()

        mag = Magnetometer()
        mag.add_stream('socket')
        robot.append(mag)
        mag.properties(date = 2015.0)

        teleport = Teleport()
        teleport.add_stream('socket')
        teleport.alter('geodetic')
        robot.append(teleport)

        env = Environment('empty', fastmode = True)
        env.add_service('socket')
        env.properties(longitude = 43.6, latitude = 1.4333, altitude = 135.0)

    def _teleport_and_test(self, x, y, z, mag_x, mag_y, mag_z):

        precision = 0.1
        self.teleport_stream.publish({'x': x, 'y': y, 'z' : z, 'yaw': 0.0,
                                      'pitch': 0.0, 'roll': 0.0})
        self.m.sleep(0.1)
        mag = self.mag_stream.get()
        self.assertAlmostEquals(mag['x'], mag_x, delta = precision)
        self.assertAlmostEquals(mag['y'], mag_y, delta = precision)
        self.assertAlmostEquals(mag['z'], mag_z, delta = precision)

    def test_geodetic_modifier(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            self.m = morse
            self.mag_stream = morse.robot.mag
            self.teleport_stream = morse.robot.teleport

            # Tests from WMM2015 
            self._teleport_and_test(0, 80, 0,  6627.1, -445.9, 54432.3)
            self._teleport_and_test(120, 0, 0, 39518.2,  392.9, -11252.4)
            self._teleport_and_test(240, -80, 0, 5797.3, 15761.1, -52919.1)
            self._teleport_and_test(0, 80, 100000,  6314.3, -471.6,  52269.8)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(MagnemoterTest)
