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
import math
from pymorse import Morse


class OrientationTest(MorseTestCase):

    def setUpEnv(self):
        
        robot = RMax('robot')
        robot.translate(10.0, 8.0, 20.0)
        
        attitude = Attitude()
        attitude.add_stream('socket')
        attitude.properties(ComputationMode = 'Velocity')
        robot.append(attitude)

        attitude_pos = Attitude()
        attitude_pos.add_stream('socket')
        attitude_pos.properties(ComputationMode = 'Position')
        robot.append(attitude_pos)

        orientation = Orientation('orientation')
        orientation.add_stream('socket')
        orientation.properties(speed = 0.5)
        robot.append(orientation)

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def _test_attitude(self, yaw, pitch, roll, wx, wy, wz):
        precision = 0.05

        attitude = self.att_stream.get()
        attitude_pos = self.att_pos_stream.last()

        self.assertAlmostEqual(attitude['rotation']['yaw'], yaw, delta = precision)
        self.assertAlmostEqual(attitude['rotation']['pitch'], pitch, delta = precision)
        self.assertAlmostEqual(attitude['rotation']['roll'], roll, delta = precision)

        self.assertAlmostEqual(attitude_pos['rotation']['yaw'], yaw, delta = precision)
        self.assertAlmostEqual(attitude_pos['rotation']['pitch'], pitch, delta = precision)
        self.assertAlmostEqual(attitude_pos['rotation']['roll'], roll, delta = precision)

        self.assertAlmostEqual(attitude['angular_velocity'][0], wx, delta = precision)
        self.assertAlmostEqual(attitude['angular_velocity'][1], wy, delta = precision)
        self.assertAlmostEqual(attitude['angular_velocity'][2], wz, delta = precision)

        self.assertAlmostEqual(attitude_pos['angular_velocity'][0], wx, delta = precision)
        self.assertAlmostEqual(attitude_pos['angular_velocity'][1], wy, delta = precision)
        self.assertAlmostEqual(attitude_pos['angular_velocity'][2], wz, delta = precision)

    def send_angles(self, yaw, pitch, roll):
        self.orientation_stream.publish({'yaw' : yaw, 'pitch' : pitch, 'roll' : roll})

    def test_orientation(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            self.att_stream = morse.robot.attitude
            self.att_pos_stream = morse.robot.attitude_pos
            self.orientation_stream = morse.robot.orientation

            self._test_attitude(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

            self.send_angles(3.0, 0.0, 0.0)
            morse.sleep(1.0)

            self._test_attitude(0.5, 0.0, 0.0, 0.0, 0.0, 0.5)

            morse.sleep(6.0)

            self._test_attitude(3.0, 0.0, 0.0, 0.0, 0.0, 0.0)

            # Be smart on the rotation order
            self.send_angles(-3.0, 0.0, 0.0)
            morse.sleep(1.0)
            self._test_attitude(-3.0, 0.0, 0.0, 0.0, 0.0, 0.0)

            # Turn in the order direction
            self.send_angles(2.0, 0.0, 0.0)
            morse.sleep(1.0)
            self._test_attitude(2.8, 0.0, 0.0, 0.0, 0.0, -0.5)

            morse.sleep(2.0)
            self._test_attitude(2.0, 0.0, 0.0, 0.0, 0.0, 0.0)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(OrientationTest)
