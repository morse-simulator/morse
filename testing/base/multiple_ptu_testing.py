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

def send_angles(s, pan, tilt):
    s.publish({'pan' : pan, 'tilt' : tilt})

class MultiplePTUTest(MorseTestCase):
    """
    Non regression test for #536

    PTU related component are really tested in ptu_testing.py
    """

    def setUpEnv(self):

        ptu_x = 0.2020
        ptu_z = 1.4400

        robot = ATRV()

        ptu0 = PTU()
        ptu0.add_stream('socket')
        ptu0.translate(x=ptu_x, z=ptu_z)
        ptu0.add_service('socket')
        robot.append(ptu0)

        posture0 = PTUPosture()
        posture0.add_stream('socket')
        ptu0.append(posture0)

        ptu1 = PTU()
        ptu1.add_stream('socket')
        ptu1.translate(x = 2 * ptu_x, z = 2 * ptu_z)
        ptu1.add_service('socket')
        robot.append(ptu1)

        posture1 = PTUPosture()
        posture1.add_stream('socket')
        ptu1.append(posture1)

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_datastream(self):
        """ Test if we can connect to the pose data stream, and read from it.
        """

        with Morse() as morse:
            posture_stream0 = morse.robot.ptu0.posture0
            ptu_stream0 = morse.robot.ptu0
            posture_stream1 = morse.robot.ptu1.posture1
            ptu_stream1 = morse.robot.ptu1

            precision = 0.02
            moving_precision = 0.1

            posture0 = posture_stream0.get()
            posture1 = posture_stream1.get()

            self.assertAlmostEqual(posture0['pan'], 0.0, delta=precision)
            self.assertAlmostEqual(posture0['tilt'], 0.0, delta=precision)
            self.assertAlmostEqual(posture1['pan'], 0.0, delta=precision)
            self.assertAlmostEqual(posture1['tilt'], 0.0, delta=precision)

            send_angles(ptu_stream0, 1.0, 0.0)
            send_angles(ptu_stream1, -1.0, 0.0)
            morse.sleep(1.0)

            posture0 = posture_stream0.get()
            posture1 = posture_stream1.get()

            self.assertAlmostEqual(posture0['pan'], 1.0, delta=moving_precision)
            self.assertAlmostEqual(posture0['tilt'], 0.0, delta=moving_precision)
            self.assertAlmostEqual(posture1['pan'], -1.0, delta=moving_precision)
            self.assertAlmostEqual(posture1['tilt'], 0.0, delta=moving_precision)

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(MultiplePTUTest)
