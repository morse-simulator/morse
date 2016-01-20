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

import math
from pymorse import Morse

class RadarAltimeterTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = RMax('robot')
        robot.translate(0.0, 0.0, 20.0)
        
        teleport = Teleport()
        teleport.add_stream('socket')
        robot.append(teleport)

        altimeter = RadarAltimeter()
        robot.append(altimeter)
        altimeter.add_stream('socket')
        altimeter.properties(MaxRange = 25.0)

        env = Environment('indoors-1/boxes', fastmode = True)
        env.add_service('socket')

        ground = bpymorse.get_object('Ground')
        ground.game.use_actor = True

    def test_teleport(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            teleport_stream = morse.robot.teleport
            altimeter_stream = morse.robot.altimeter

            precision = 0.01

            altitude = altimeter_stream.get()
            self.assertAlmostEquals(altitude['z'], 20.0, delta = precision)

            teleport_stream.publish({'x': 0.0, 'y': 0.0, 'z': 15.0,
                'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0})
            morse.sleep(0.1)

            altitude = altimeter_stream.get()
            self.assertAlmostEquals(altitude['z'], 15.0, delta = precision)

            teleport_stream.publish({'x': 0.0, 'y': 0.0, 'z': 30.0,
                'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0})
            morse.sleep(0.1)

            altitude = altimeter_stream.get()
            self.assertEqual(altitude['z'], float('inf'))

            teleport_stream.publish({'x': 0.0, 'y': 0.0, 'z': 12.0,
                'yaw': 0.0, 'pitch': math.pi/2, 'roll': math.pi/2})
            morse.sleep(0.1)

            altitude = altimeter_stream.get()
            self.assertAlmostEquals(altitude['z'], 12.0, delta = precision)

            # Goes on top on the red box, size ~1.8m
            teleport_stream.publish({'x': -7.5, 'y': 0.0, 'z': 15.0,
                'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0})
            morse.sleep(0.1)

            altitude = altimeter_stream.get()
            self.assertAlmostEquals(altitude['z'], 13.14, delta = precision)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(RadarAltimeterTest)
