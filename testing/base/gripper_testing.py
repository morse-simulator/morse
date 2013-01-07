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

def send_pose(s, x, y, yaw):
    s.send(json.dumps({'x' : x, 'y' : y, 'z' : 0.0, \
                       'yaw' : yaw, 'pitch' : 0.0, 'roll' : 0.0}).encode())

class gripperTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        robot = ATRV()

        kuka_lwr = KukaLWR()
        robot.append(kuka_lwr)
        kuka_lwr.translate(x=0.5, z=0.9)
        kuka_lwr.configure_mw('socket')
        kuka_lwr.configure_service('socket')

        gripper = Gripper()
        gripper.translate(z=1.28)
        kuka_lwr.append(gripper)
        gripper.configure_service('socket')

        motion = Teleport()
        robot.append(motion)
        motion.configure_mw('socket')

        tape1 = PassiveObject(prefix='BlackVideotape')
        tape1.properties(Object = True, Graspable = True, Label = "BlackTape")
        tape1.translate(x=5, y=5, z=0)

        tape2 = PassiveObject(prefix='WhiteVideotape')
        tape2.properties(Object = True, Graspable = True, Label = "WhiteTapee")
        tape2.translate(x=5, y=-5, z=0)

        gripper.cfg_radar(angle=180.0, distance=2.0)

        env = Environment('empty', fastmode=True)
        env.configure_service('socket')


    def test_kuka_controller(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
            # kuka controller socket
            port = morse.get_stream_port('kuka_armature')
            kuka_client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            kuka_client.connect(('localhost', port))

            port = morse.get_stream_port('Motion_Controller')
            teleport_stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            teleport_stream.connect(('localhost', port))

            kuka_client.send(json.dumps({'kuka_1' : 0.0,
                                         'kuka_2' : -1.57,
                                         'kuka_3' : 0.0,
                                         'kuka_4' : 0.0,
                                         'kuka_5' : 0.0,
                                         'kuka_6' : 0.0,
                                         'kuka_7' : 0.0}).encode())

            obj = morse.call_server('Gripper', 'grab')
            self.assertEqual(obj, None)

            obj = morse.call_server('Gripper', 'release')
            self.assertEqual(obj, None)

            send_pose(teleport_stream, 3.0, 5, 0.0)

            obj = morse.call_server('Gripper', 'grab')
            self.assertEqual(obj, 'BlackVideotape')

            send_pose(teleport_stream, 3.0, -5.0, 0.0)
            obj = morse.call_server('Gripper', 'grab')
            self.assertEqual(obj, 'BlackVideotape')

            send_pose(teleport_stream, 3.0, 8.0, 0.0)
            obj = morse.call_server('Gripper', 'release')
            self.assertEqual(obj, True)

            send_pose(teleport_stream, 3.0, -5.0, 0.0)
            obj = morse.call_server('Gripper', 'grab')
            self.assertEqual(obj, 'WhiteVideotape')

            obj = morse.call_server('Gripper', 'release')
            self.assertEqual(obj, True)

            send_pose(teleport_stream, 3.0, 8.0, 0.0)
            obj = morse.call_server('Gripper', 'grab')
            self.assertEqual(obj, 'BlackVideotape')


########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(gripperTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

