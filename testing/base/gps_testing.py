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


class GPSTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = ATRV()
        robot.translate(10.0, 8.0, 0.0)
        
        gps = GPS()
        gps.add_stream('socket')
        robot.append(gps)

        gps_raw = GPS()
        gps_raw.level('raw')
        gps_raw.add_stream('socket')
        robot.append(gps_raw)

        teleport = Teleport()
        robot.append(teleport)
        teleport.add_stream('socket')

        env = Environment('empty', fastmode = True)
        env.add_service('socket')
        env.properties(longitude = 43.6, latitude = 1.4333, altitude = 135.0)

    def test_read_gps(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            gps_stream = morse.robot.gps
            gps_raw_stream = morse.robot.gps_raw
            teleport_stream = morse.robot.teleport

            pos = gps_stream.get()
            pos_raw = gps_raw_stream.get()

            precision = 0.02
            precision_deg = 0.0001
            self.assertAlmostEqual(pos['x'], 10.0, delta=precision)
            self.assertAlmostEqual(pos['y'], 8.0, delta=precision)
            # Z = 0.1 : pose of the ATRV's center relative to the world
            self.assertAlmostEqual(pos['z'], 0.1, delta=precision)

            # Values get from the sensor, just to check that there are
            # no regression134.88
            self.assertAlmostEqual(pos_raw['latitude'], 1.4333, delta=precision_deg)
            self.assertAlmostEqual(pos_raw['longitude'], 43.6, delta=precision_deg)
            self.assertAlmostEqual(pos_raw['altitude'], 135.1, delta=precision)

            teleport_stream.publish({'x' : 100.0, 'y' : 200.0, 'z' : 50.0, 'yaw' : 0.0, 'pitch' : 0.0, 'roll' : 0.0})
            morse.sleep(0.01)


            pos = gps_stream.get()
            pos_raw = gps_raw_stream.last()

            self.assertAlmostEqual(pos['x'], 100.0, delta=precision)
            self.assertAlmostEqual(pos['y'], 200.0, delta=precision)
            self.assertAlmostEqual(pos['z'], 50.0, delta=precision)

            # Values get from the sensor, just to check that there are
            # no regression
            self.assertAlmostEqual(pos_raw['latitude'], 1.4351, delta=precision_deg)
            self.assertAlmostEqual(pos_raw['longitude'], 43.6009, delta=precision_deg)
            self.assertAlmostEqual(pos_raw['altitude'], 185.003, delta=precision)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(GPSTest)
