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
from pymorse import Morse

# Include this import to be able to use your test file as a regular
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class KUKA_LWR_Test(MorseTestCase):
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

    def test_kuka_controller(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
            # Read the armature position
            posture_stream = morse.stream('kuka_posture')
            posture = posture_stream.get()
            # Test each of the fields individually
            self.assertAlmostEqual(posture['x'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['y'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['z'], 1.0, delta=0.02)
            self.assertAlmostEqual(posture['yaw'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['pitch'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['roll'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_1'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_2'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_3'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_4'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_5'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_6'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_7'], 0.0, delta=0.02)


            # kuka controller socket
            port = morse.get_stream_port('kuka_armature')
            kuka_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            kuka_client.connect(('localhost', port))

            kuka_client.send(json.dumps({'kuka_1' : 1.57,
                                         'kuka_2' : 2.0,
                                         'kuka_3' : 1.0,
                                         'kuka_4' : 4.0,
                                         'kuka_5' : 1.0,
                                         'kuka_6' : -2.0,
                                         'kuka_7' : 1.0}).encode())
            sleep(2)

            posture = posture_stream.get()
            self.assertAlmostEqual(posture['kuka_1'], 1.57, delta=0.02)
            self.assertAlmostEqual(posture['kuka_2'], 2.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_3'], 1.0, delta=0.02)
            # Angles larger than PI are transformed
            self.assertAlmostEqual(posture['kuka_4'], -2.28, delta=0.02)
            self.assertAlmostEqual(posture['kuka_5'], 1.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_6'], -2.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_7'], 1.0, delta=0.02)


    def test_kuka_service_controller(self):
        # Wait a bit for the ports for the previous test to be closed
        with Morse() as morse:

            # Read the start position, it must be (0.0, 0.0, 0.0)
            posture_stream = morse.stream('kuka_posture')
            posture = posture_stream.get()
            self.assertAlmostEqual(posture['kuka_1'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_2'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_3'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_4'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_5'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_6'], 0.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_7'], 0.0, delta=0.02)

            morse.call_server('kuka_armature', 'set_rotation_array', 1.57, 2.0, 1.0, -2.28, 1.0, -2.0, 1.0)
            sleep(2)

            posture = posture_stream.get()
            self.assertAlmostEqual(posture['kuka_1'], 1.57, delta=0.02)
            self.assertAlmostEqual(posture['kuka_2'], 2.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_3'], 1.0, delta=0.02)
            # Angles larger than PI are transformed
            self.assertAlmostEqual(posture['kuka_4'], -2.28, delta=0.02)
            self.assertAlmostEqual(posture['kuka_5'], 1.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_6'], -2.0, delta=0.02)
            self.assertAlmostEqual(posture['kuka_7'], 1.0, delta=0.02)


            # XXX need to test other services offered by kuka
            # controller, but pymorse support is not good enough at the
            # moment

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(KUKA_LWR_Test)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

