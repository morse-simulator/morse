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
import time
from pymorse import Morse


class BatteryTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = ATRV()

        battery = Battery()
        battery.add_stream('socket')
        battery.properties(DischargingRate = 10.0)
        robot.append(battery)

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_read_battery(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            bat_stream = morse.robot.battery

            bat = bat_stream.get()
            cur_bat = bat['charge']
            time.sleep(2.0)

            bat = bat_stream.get()
            # Can't be really precise as we don't have exact timestamp
            # about when we get data
            self.assertAlmostEqual(bat['charge'] - cur_bat, -20.0, delta=0.5)
            cut_bat = bat['charge']

            # Now the battery must be empty
            time.sleep(10.0)
            bat = bat_stream.get()
            self.assertAlmostEqual(bat['charge'], 0.0, delta=0.001)



########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(BatteryTest)

