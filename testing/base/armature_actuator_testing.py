#! /usr/bin/env python
"""
This script tests the KUKA LWR arm, both the data and service api
"""

import sys
import socket
import json
import math
from time import sleep
from morse.testing.testing import MorseTestCase
from pymorse import Morse, MorseServerError

# Include this import to be able to use your test file as a regular
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class ArmatureActuatorTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        robot = Robot('atrv')

        kuka_lwr = Actuator('kuka_lwr')
        robot.append(kuka_lwr)
        kuka_lwr.translate(z=0.9)
        kuka_lwr.configure_mw('socket')
        kuka_lwr.configure_service('socket')

        kuka_posture = Sensor('kuka_posture')
        robot.append(kuka_posture)
        kuka_posture.configure_mw('socket')

        env = Environment('indoors-1/indoor-1')
        env.configure_service('socket')

    def test_armature_actuator(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
            # Read the armature position
            posture_stream = morse.stream('kuka_posture')
            posture = posture_stream.get()

            precision = 0.02

            # Test each of the fields individually
            self.assertAlmostEqual(posture['x'], 0.0, delta=precision)
            self.assertAlmostEqual(posture['y'], 0.0, delta=precision)
            self.assertAlmostEqual(posture['z'], 1.0, delta=precision)
            self.assertAlmostEqual(posture['yaw'], 0.0, delta=precision)
            self.assertAlmostEqual(posture['pitch'], 0.0, delta=precision)
            self.assertAlmostEqual(posture['roll'], 0.0, delta=precision)
            self.assertAlmostEqual(posture['kuka_1'], 0.0, delta=precision)
            self.assertAlmostEqual(posture['kuka_2'], 0.0, delta=precision)
            self.assertAlmostEqual(posture['kuka_3'], 0.0, delta=precision)
            self.assertAlmostEqual(posture['kuka_4'], 0.0, delta=precision)
            self.assertAlmostEqual(posture['kuka_5'], 0.0, delta=precision)
            self.assertAlmostEqual(posture['kuka_6'], 0.0, delta=precision)
            self.assertAlmostEqual(posture['kuka_7'], 0.0, delta=precision)


            channels = morse.call_server('kuka_armature', 'get_channels')
            self.assertEqual(channels, ['kuka_1', 'kuka_2', 'kuka_3', 'kuka_4', 'kuka_5', 'kuka_6', 'kuka_7'])

            res = morse.call_server('kuka_armature', 'get_rotations')
            for channel in channels:
                for i in range(3):
                    self.assertAlmostEqual(res[channel][i], 0.0, delta=precision)

            res = morse.call_server('kuka_armature', 'get_rotation', 'kuka_5')
            for i in range(3):
                self.assertAlmostEqual(res[i], 0.0, delta=precision)

            with self.assertRaises(MorseServerError):
                res = morse.call_server('kuka_armature', 'get_rotation', 'pipo')

            # Not sure to understand well these values
            # Must check they are completly correct

            res = morse.call_server('kuka_armature', 'get_dofs')
            self.assertEqual(res['kuka_1'], [0, 1, 0])
            self.assertEqual(res['kuka_2'], [0, 0, 1])
            self.assertEqual(res['kuka_3'], [0, 1, 0])
            self.assertEqual(res['kuka_4'], [0, 0, 1])
            self.assertEqual(res['kuka_5'], [0, 1, 0])
            self.assertEqual(res['kuka_6'], [0, 0, 1])
            self.assertEqual(res['kuka_7'], [0, 1, 0])

            # get_IK_limits is a synonym of get_dofs
            res2 = morse.call_server('kuka_armature', 'get_IK_limits')
            self.assertEqual(res, res2)

            res = morse.call_server('kuka_armature', 'get_channel_lengths')
            self.assertAlmostEqual(res['kuka_1'], 0.31, delta=precision)
            self.assertAlmostEqual(res['kuka_2'], 0.20, delta=precision)
            self.assertAlmostEqual(res['kuka_3'], 0.20, delta=precision)
            self.assertAlmostEqual(res['kuka_4'], 0.20, delta=precision)
            self.assertAlmostEqual(res['kuka_5'], 0.19, delta=precision)
            self.assertAlmostEqual(res['kuka_6'], 0.08, delta=precision)
            self.assertAlmostEqual(res['kuka_7'], 0.13, delta=precision)

            res = morse.call_server('kuka_armature', 'get_robot_parent_name')
            self.assertEqual(res, "ATRV")


            # Move the arm now, and get the measure 
            morse.call_server('kuka_armature', 'set_rotation_array', 1.57, 2.0, 1.0, -2.28, 1.0, -2.0, 1.0)
            sleep(2)

            posture = posture_stream.get()
            self.assertAlmostEqual(posture['kuka_1'], 1.57, delta=precision)
            self.assertAlmostEqual(posture['kuka_2'], 2.0, delta=precision)
            self.assertAlmostEqual(posture['kuka_3'], 1.0, delta=precision)
            self.assertAlmostEqual(posture['kuka_4'], -2.28, delta=precision)
            self.assertAlmostEqual(posture['kuka_5'], 1.0, delta=precision)
            self.assertAlmostEqual(posture['kuka_6'], -2.0, delta=precision)
            self.assertAlmostEqual(posture['kuka_7'], 1.0, delta=precision)

            res = morse.call_server('kuka_armature', 'get_rotations')
            self.assertAlmostEqual(res['kuka_1'][0], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_1'][1], 1.57, delta=precision)
            self.assertAlmostEqual(res['kuka_1'][2], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_2'][0], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_2'][1], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_2'][2], 2.0, delta=precision)
            self.assertAlmostEqual(res['kuka_3'][0], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_3'][1], 1.0, delta=precision)
            self.assertAlmostEqual(res['kuka_3'][2], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_4'][0], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_4'][1], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_4'][2], -2.28, delta=precision)
            self.assertAlmostEqual(res['kuka_5'][0], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_5'][1], 1.0, delta=precision)
            self.assertAlmostEqual(res['kuka_5'][2], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_6'][0], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_6'][1], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_6'][2], -2.0, delta=precision)
            self.assertAlmostEqual(res['kuka_7'][0], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_7'][1], 1.0, delta=precision)
            self.assertAlmostEqual(res['kuka_7'][2], 0.0, delta=precision)

            # Here, in fact, we test kuka_lwr::set_rotation, not
            # armature_actuator::set_rotation. See bug #86.
            # Invalid channel
            with self.assertRaises(MorseServerError):
                morse.call_server('kuka_armature', 'set_rotation', 'pipo', [2.0, 3.0, 4.0])

            # Bad number of args
            with self.assertRaises(MorseServerError):
                morse.call_server('kuka_armature', 'set_rotation', 'kuka_5')

            morse.call_server('kuka_armature', 'set_rotation', 'kuka_5', [0.0, math.pi/2, 0.0])
            sleep(1)

            # Only kuka_5 has changed to the value math.pi/2
            res = morse.call_server('kuka_armature', 'get_rotations')
            self.assertAlmostEqual(res['kuka_1'][0], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_1'][1], 1.57, delta=precision)
            self.assertAlmostEqual(res['kuka_1'][2], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_2'][0], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_2'][1], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_2'][2], 2.0, delta=precision)
            self.assertAlmostEqual(res['kuka_3'][0], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_3'][1], 1.0, delta=precision)
            self.assertAlmostEqual(res['kuka_3'][2], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_4'][0], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_4'][1], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_4'][2], -2.28, delta=precision)
            self.assertAlmostEqual(res['kuka_5'][0], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_5'][1], math.pi/2, delta=precision)
            self.assertAlmostEqual(res['kuka_5'][2], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_6'][0], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_6'][1], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_6'][2], -2.0, delta=precision)
            self.assertAlmostEqual(res['kuka_7'][0], 0.0, delta=precision)
            self.assertAlmostEqual(res['kuka_7'][1], 1.0, delta=precision)
            self.assertAlmostEqual(res['kuka_7'][2], 0.0, delta=precision)

            # Injecting value on angles non free has no impact
            morse.call_server('kuka_armature', 'set_rotation', 'kuka_5', [1.0, math.pi/2, 1.0])
            sleep(1)

            res = morse.call_server('kuka_armature', 'get_rotation', 'kuka_5')
            self.assertAlmostEqual(res[0], 0.0, delta=precision)
            self.assertAlmostEqual(res[1], math.pi/2, delta=precision)
            self.assertAlmostEqual(res[0], 0.0, delta=precision)

            # Injecting value > max must be rejected or limited to max
            # See bug #87 on Morse GitHub
            res = morse.call_server('kuka_armature', 'get_IK_minmax')
            max = res['kuka_5'][1][1]
            morse.call_server('kuka_armature', 'set_rotation', 'kuka_5', [0.0, max + 0.1, 0.0])
            sleep(1)

            res = morse.call_server('kuka_armature', 'get_rotation', 'kuka_5')
            self.assertAlmostEqual(res[0], 0.0, delta=precision)
            self.assertAlmostEqual(res[1], max, delta=precision)
            self.assertAlmostEqual(res[0], 0.0, delta=precision)



########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(ArmatureActuatorTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

