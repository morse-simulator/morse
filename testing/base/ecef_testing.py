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


class ECEFModifierTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = ATRV()
        robot.translate(10.0, 8.0, 0.0)
        
        gps = GPS()
        gps.add_stream('socket')
        robot.append(gps)

        gps_mod = GPS()
        gps_mod.add_stream('socket')
        gps_mod.alter('ECEF')
        robot.append(gps_mod)

        teleport = Teleport()
        teleport.add_stream('socket')
        robot.append(teleport)

        teleport_mod = Teleport()
        teleport_mod.add_stream('socket')
        robot.append(teleport_mod)
        teleport_mod.alter('ECEF')

        env = Environment('empty', fastmode = True)
        env.add_service('socket')
        env.properties(longitude = 43.6, latitude = 1.4333, altitude = 135.0)


    def test_ecef_modifier(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            gps_stream = morse.robot.gps
            gps_mod_stream = morse.robot.gps_mod
            teleport_stream = morse.robot.teleport
            teleport_mod_stream = morse.robot.teleport_mod

            pos = gps_stream.get()
            pos_mod = gps_mod_stream.get()

            precision = 0.02
            self.assertAlmostEqual(pos['x'], 10.0, delta=precision)
            self.assertAlmostEqual(pos['y'], 8.0, delta=precision)
            # Z = 0.1 : pose of the ATRV's center relative to the world
            self.assertAlmostEqual(pos['z'], 0.1, delta=precision)

            self.assertAlmostEqual(pos_mod['x'], 4617522.748, delta=precision)
            self.assertAlmostEqual(pos_mod['y'], 4397221.061, delta=precision)
            self.assertAlmostEqual(pos_mod['z'], 158481.296875, delta=precision)

            teleport_stream.publish({'x' : 100.0, 'y' : 200.0, 'z' : 50.0, 'yaw' : 0.0, 'pitch' : 0.0, 'roll' : 0.0})
            morse.sleep(0.01)

            pos = gps_stream.get()
            pos_mod = gps_mod_stream.last()

            self.assertAlmostEqual(pos['x'], 100.0, delta=precision)
            self.assertAlmostEqual(pos['y'], 200.0, delta=precision)
            self.assertAlmostEqual(pos['z'], 50.0, delta=precision)

            self.assertAlmostEqual(pos_mod['x'], 4617493.329 , delta=precision)
            self.assertAlmostEqual(pos_mod['y'], 4397317.326, delta=precision)
            self.assertAlmostEqual(pos_mod['z'], 158674.47893, delta=precision)

            morse.deactivate('robot.teleport')
            teleport_mod_stream.publish({'x': 4617522.748, 'y': 4397221.061, 'z':158481.296875,  'yaw' : 0.0, 'pitch' : 0.0, 'roll': 0.0})
            morse.sleep(0.03)

            pos = gps_stream.get()
            pos_mod = gps_mod_stream.last()

            self.assertAlmostEqual(pos['x'], 10.0, delta=precision)
            self.assertAlmostEqual(pos['y'], 8.0, delta=precision)
            self.assertAlmostEqual(pos['z'], 0.1, delta=precision)

            self.assertAlmostEqual(pos_mod['x'], 4617522.748, delta=precision)
            self.assertAlmostEqual(pos_mod['y'], 4397221.061, delta=precision)
            self.assertAlmostEqual(pos_mod['z'], 158481.296875, delta=precision)



########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(ECEFModifierTest)
