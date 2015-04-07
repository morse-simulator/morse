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

def send_pose(s, x, y, z):
    s.publish({'x' : x, 'y' : y,  'z' : z, 'yaw' : 0.0, 'pitch' : 0.0, 'roll' : 0.0})

class BarometerTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = Morsy()

        barometer = Barometer()
        barometer.add_stream('socket')
        barometer.properties(DischargingRate = 10.0)
        robot.append(barometer)

        teleport = Teleport()
        teleport.add_stream('socket')
        robot.append(teleport)

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_read_barometer(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            bat_stream = morse.robot.barometer
            teleport_stream = morse.robot.teleport

            bat = bat_stream.get()
            self.assertAlmostEqual(bat['pressure'], 101325.0, delta = 0.1)

            # pressure is independant of position (x,y)
            send_pose(teleport_stream, 5.0, 2.0, 0.0)
            morse.sleep(0.01)
            bat = bat_stream.get()
            self.assertAlmostEqual(bat['pressure'], 101325.0, delta = 0.1)

            # Pressure computed from
            # http://www.digitaldutch.com/atmoscalc/

            send_pose(teleport_stream, 0.0, 0.0, 100.0)
            morse.sleep(0.01)
            bat = bat_stream.get()
            self.assertAlmostEqual(bat['pressure'], 100129.0, delta = 0.1)

            send_pose(teleport_stream, 0.0, 0.0, 1000.0)
            morse.sleep(0.01)
            bat = bat_stream.get()
            self.assertAlmostEqual(bat['pressure'], 89871.0, delta = 0.1)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(BarometerTest)

