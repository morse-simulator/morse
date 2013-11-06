#! /usr/bin/env python
"""
This script tests the KUKA LWR arm, both the data and service api
"""

import sys
from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

def send_pose(s, morse, x, y, yaw):
    s.publish({'x' : x, 'y' : y, 'z' : 0.0, \
               'yaw' : yaw, 'pitch' : 0.0, 'roll' : 0.0})
    morse.sleep(0.1)

class gripperTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        robot = ATRV('robot')

        arm = KukaLWR()
        robot.append(arm)
        arm.translate(x=0.5, z=0.9)
        arm.add_stream('socket')
        arm.add_service('socket')

        gripper = Gripper('gripper')
        gripper.translate(z=1.28)
        arm.append(gripper)
        gripper.properties(Angle = 180.0, Distance=2.0)
        gripper.add_service('socket')

        teleport = Teleport()
        robot.append(teleport)
        teleport.add_stream('socket')

        tape1 = PassiveObject(prefix='BlackVideotape')
        tape1.properties(Object = True, Graspable = True, Label = "BlackTape")
        tape1.translate(x=5, y=5, z=0)

        tape2 = PassiveObject(prefix='WhiteVideotape')
        tape2.properties(Object = True, Graspable = True, Label = "WhiteTapee")
        tape2.translate(x=5, y=-5, z=0)

        env = Environment('empty', fastmode=True)
        env.add_service('socket')


    def test_gripper(self):
        """ This test is guaranteed to be started only when the simulator
        is ready.
        """
        with Morse() as morse:
            # kuka controller socket
            kuka_client = morse.robot.arm
            teleport_stream = morse.robot.teleport

            kuka_client.set_rotation('kuka_2', -1.57)
            morse.sleep(0.1)

            obj = morse.robot.arm.gripper.grab().result()
            self.assertEqual(obj, None)

            obj = morse.robot.arm.gripper.release().result()
            self.assertEqual(obj, None)

            send_pose(teleport_stream, morse, 3.0, 5, 0.0)
            morse.sleep(0.1)

            obj = morse.robot.arm.gripper.grab().result()
            self.assertEqual(obj, 'tape1')

            send_pose(teleport_stream, morse, 3.0, -5.0, 0.0)
            obj = morse.robot.arm.gripper.grab().result()
            self.assertEqual(obj, 'tape1')

            send_pose(teleport_stream, morse, 3.0, 8.0, 0.0)
            obj = morse.robot.arm.gripper.release().result()
            self.assertEqual(obj, True)

            send_pose(teleport_stream, morse, 3.0, -5.0, 0.0)
            obj = morse.robot.arm.gripper.grab().result()
            self.assertEqual(obj, 'tape2')

            obj = morse.robot.arm.gripper.release().result()
            self.assertEqual(obj, True)

            send_pose(teleport_stream, morse, 3.0, 8.0, 0.0)
            obj = morse.robot.arm.gripper.grab().result()
            self.assertEqual(obj, 'tape1')


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(gripperTest)
