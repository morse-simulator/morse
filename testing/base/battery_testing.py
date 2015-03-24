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

def send_pose(s, x, y):
    s.publish({'x' : x, 'y' : y, 'z' : 0.0, 'yaw' : 0.0, 'pitch' : 0.0, 'roll' : 0.0})

class BatteryTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = ATRV()

        battery = Battery()
        battery.add_stream('socket')
        battery.properties(DischargingRate = 10.0)
        robot.append(battery)

        teleport = Teleport()
        teleport.add_stream('socket')
        robot.append(teleport)

        charging_zone = Zone(type = 'Charging')
        charging_zone.size = [5.0, 5.0, 5.0]
        charging_zone.translate(x = 10.0, z = 2.0)

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_read_battery(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            bat_stream = morse.robot.battery
            teleport_stream = morse.robot.teleport

            bat = bat_stream.get()
            cur_bat = bat['charge']
            morse.sleep(2.0)

            bat = bat_stream.get()
            # Can't be really precise as we don't have exact timestamp
            # about when we get data
            self.assertAlmostEqual(bat['charge'] - cur_bat, -20.0, delta=0.5)
            cut_bat = bat['charge']
            self.assertEqual(bat['status'], 'Discharging')

            # Now the battery must be empty
            morse.sleep(10.0)
            bat = bat_stream.get()
            self.assertAlmostEqual(bat['charge'], 0.0, delta=0.001)
            self.assertEqual(bat['status'], 'Discharging')

            # Teleport in the charging zone and check the battery charge
            # grows up
            send_pose(teleport_stream, 7.0, 0.0)
            morse.sleep(2.0)
            bat = bat_stream.get()
            self.assertAlmostEqual(bat['charge'], 20.0, delta=0.5)
            self.assertEqual(bat['status'], 'Charging')

            # Teleport out of the charging zone, the battery charge must
            # decrease
            send_pose(teleport_stream, 2.0, 0.0)
            morse.sleep(2.5)
            bat = bat_stream.get()
            self.assertAlmostEqual(bat['charge'], 0.0, delta=0.001)
            self.assertEqual(bat['status'], 'Discharging')


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(BatteryTest)

