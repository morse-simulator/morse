#! /usr/bin/env python
"""
This script tests the KUKA LWR arm, both the data and service api
"""

import sys
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
    s.publish({'x' : x, 'y' : y, 'z' : 0.0, \
               'yaw' : yaw, 'pitch' : 0.0, 'roll' : 0.0})
    sleep(0.1)

class gripperTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        robot = ATRV('robot')

        kuka_lwr = KukaLWR('kuka')
        robot.append(kuka_lwr)
        kuka_lwr.translate(x=0.5, z=0.9)
        kuka_lwr.configure_mw('socket')
        kuka_lwr.configure_service('socket')

        gripper = Gripper('gripper')
        gripper.translate(z=1.28)
        kuka_lwr.append(gripper)
        gripper.configure_service('socket')

        motion = Teleport('teleport')
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


    def test_gripper(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
            # kuka controller socket
            kuka_client = morse.robot.kuka
            teleport_stream = morse.robot.teleport

            kuka_client.set_rotation('kuka_2', -1.57)
            sleep(0.1)

            obj = morse.robot.gripper.grab().result()
            self.assertEqual(obj, None)

            obj = morse.robot.gripper.release().result()
            self.assertEqual(obj, None)

            send_pose(teleport_stream, 3.0, 5, 0.0)
            sleep(0.1)

            obj = morse.robot.gripper.grab().result()
            self.assertEqual(obj, 'BlackVideotape')

            send_pose(teleport_stream, 3.0, -5.0, 0.0)
            sleep(0.1)
            obj = morse.robot.gripper.grab().result()
            self.assertEqual(obj, 'BlackVideotape')

            send_pose(teleport_stream, 3.0, 8.0, 0.0)
            sleep(0.1)
            obj = morse.robot.gripper.release().result()
            self.assertEqual(obj, True)

            send_pose(teleport_stream, 3.0, -5.0, 0.0)
            sleep(0.1)
            obj = morse.robot.gripper.grab().result()
            self.assertEqual(obj, 'WhiteVideotape')

            obj = morse.robot.gripper.release().result()
            self.assertEqual(obj, True)

            send_pose(teleport_stream, 3.0, 8.0, 0.0)
            sleep(0.1)
            obj = morse.robot.gripper.grab().result()
            self.assertEqual(obj, 'BlackVideotape')


########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(gripperTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

