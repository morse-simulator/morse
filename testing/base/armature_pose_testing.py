#! /usr/bin/env python
"""
This script tests the KUKA LWR arm, both the data and service api
"""

import sys
import math
from morse.testing.testing import MorseTestCase
from pymorse import Morse, MorseServiceFailed

# Include this import to be able to use your test file as a regular
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

class ArmaturePoseTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        robot = ATRV('robot')

        kuka_lwr = KukaLWR('arm')
        robot.append(kuka_lwr)
        kuka_lwr.translate(z=0.9)
        kuka_lwr.add_service('socket')

        kuka_posture = ArmaturePose('arm_pose')
        kuka_lwr.append(kuka_posture)
        kuka_posture.add_stream('socket')
        kuka_posture.add_service('socket')

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def test_pose_services(self):

        precision = 0.02
        JOINTS = ['kuka_1', 'kuka_2', 'kuka_3', 'kuka_4', 'kuka_5', 'kuka_6', 'kuka_7']

        with Morse() as simu:
            self.assertEqual(simu.robot.arm.arm_pose.get_joints(), JOINTS)

            #res = simu.robot.arm.get_rotations()
            #for joint in joints:
            #    for i in range(3):
            #        self.assertAlmostEqual(res[joint][i], 0.0, delta=precision)

            #res = simu.robot.arm.get_rotation('kuka_5').result()
            #for i in range(3):
            #    self.assertAlmostEqual(res[i], 0.0, delta=precision)

            #res = simu.robot.arm.get_rotation('pipo')
            #self.assertEqual(type(res.exception(2)), MorseServiceFailed)

            res = simu.robot.arm.arm_pose.get_joints_length().result()
            self.assertAlmostEqual(res['kuka_1'], 0.31, delta=precision)
            self.assertAlmostEqual(res['kuka_2'], 0.20, delta=precision)
            self.assertAlmostEqual(res['kuka_3'], 0.20, delta=precision)
            self.assertAlmostEqual(res['kuka_4'], 0.20, delta=precision)
            self.assertAlmostEqual(res['kuka_5'], 0.19, delta=precision)
            self.assertAlmostEqual(res['kuka_6'], 0.08, delta=precision)
            self.assertAlmostEqual(res['kuka_7'], 0.13, delta=precision)

            # Move the arm now, and get the measure 
            angles = [1.57, 2.0, 1.0, -1.28, 1.1, -2.0, 1.0]
            simu.robot.arm.set_rotations(angles)
            simu.sleep(0.1)

            pose = simu.robot.arm.arm_pose.get_state().result()

            target = dict(zip(JOINTS, angles))
            
            for j, v in pose.items():
                self.assertAlmostEqual(v, target[j], delta=precision)

    def test_pose_stream(self):

        precision = 0.02
        JOINTS = ['kuka_1', 'kuka_2', 'kuka_3', 'kuka_4', 'kuka_5', 'kuka_6', 'kuka_7']

        with Morse() as simu:

            pose = simu.robot.arm.arm_pose.get()

            # 7 joints + timestamp
            self.assertEqual(len(pose), 7 + 1) 

            self.assertEqual(set(pose.keys()), set(JOINTS).union(set(['timestamp'])))

            for j, v in pose.items():
                if j != 'timestamp':
                    self.assertAlmostEqual(v, 0.0, delta=precision)

            simu.robot.arm.set_rotation("kuka_2", 1).result()
            simu.sleep(0.1)
            pose = simu.robot.arm.arm_pose.get()

            self.assertAlmostEqual(simu.robot.arm.arm_pose.get()["kuka_2"], 1.0, delta = precision)

            for j, v in pose.items():
                if j != "kuka_2" and j != 'timestamp':
                    self.assertAlmostEqual(v, 0.0, delta=precision)


            # Move the arm now, and get the measure 
            angles = [1.57, 2.0, 1.0, -1.28, 1.0, -2.0, 1.0]
            simu.robot.arm.set_rotations(angles)
            simu.sleep(0.1)

            target = dict(zip(JOINTS, angles))
            pose = simu.robot.arm.arm_pose.get()
            for j, v in pose.items():
                if j != 'timestamp':
                    self.assertAlmostEqual(v, target[j], delta=precision)


            angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            simu.robot.arm.set_rotations(angles)
            simu.sleep(0.1)

            target = dict(zip(JOINTS, angles))
            pose = simu.robot.arm.arm_pose.get()
            for j, v in pose.items():
                if j != 'timestamp':
                    self.assertAlmostEqual(v, target[j], delta=precision)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(ArmaturePoseTest)
