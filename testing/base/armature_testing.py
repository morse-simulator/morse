#! /usr/bin/env python
"""
This script tests the KUKA LWR arm, both the data and service api
"""

import sys
import math
import time
from morse.testing.testing import MorseTestCase
from pymorse import Morse, MorseServiceFailed

# Include this import to be able to use your test file as a regular
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

JOINTS = ['kuka_1', 'kuka_2', 'kuka_3', 'kuka_4', 'kuka_5', 'kuka_6', 'kuka_7']
class ArmatureTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        robot = ATRV()

        arm = KukaLWR()
        robot.append(arm)
        arm.translate(z=0.9)
        arm.add_stream('socket')
        arm.add_service('socket')

        pose = Pose()
        pose.add_stream('socket')
        pose.translate(z=1.3)
        arm.append(pose)

        arm_pose = ArmaturePose()
        arm.append(arm_pose)
        arm_pose.add_stream('socket')

        motion = Teleport()
        robot.append(motion)
        motion.add_service('socket')

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def checkstate(self, simu, angles, precision = 0.025):

        pose = simu.robot.arm.arm_pose.get()
        target = dict(zip(JOINTS, angles))

        for j, v in pose.items():
            self.assertAlmostEqual(v, target[j], delta=precision)


    def test_object_attach(self):
        """ Checks that attached object are indeed attached at the right place.
        """
        precision = 0.02

        with Morse() as simu:

            self.assertAlmostEqual(simu.robot.arm.pose.get()['z'], 2.3, delta = 0.01)
            self.assertAlmostEqual(simu.robot.arm.pose.get()['x'], 0.0, delta = 0.01)
            self.assertAlmostEqual(simu.robot.arm.pose.get()['y'], 0.0, delta = 0.01)
            self.assertAlmostEqual(simu.robot.arm.pose.get()['pitch'], 0.0, delta = 0.01)
            simu.robot.motion.translate(1.0)
            time.sleep(0.1)
            self.assertAlmostEqual(simu.robot.arm.pose.get()['z'], 2.3, delta = 0.01)
            self.assertAlmostEqual(simu.robot.arm.pose.get()['x'], 1.0, delta = 0.01)
            self.assertAlmostEqual(simu.robot.arm.pose.get()['y'], 0.0, delta = 0.01)
            self.assertAlmostEqual(simu.robot.arm.pose.get()['pitch'], 0.0, delta = 0.01)
            simu.robot.arm.set_rotation("kuka_2", math.radians(-90)).result()
            time.sleep(0.1)
            self.assertAlmostEqual(simu.robot.arm.pose.get()['z'], 1.31, delta = 0.01)
            self.assertAlmostEqual(simu.robot.arm.pose.get()['x'], 1.99, delta = 0.01)
            self.assertAlmostEqual(simu.robot.arm.pose.get()['pitch'], math.radians(90), delta = 0.01)

    def test_immediate_api(self):
        """ Tests the services that have an immediate result
        (no speed limit taken into account)

        """
        # TODO: check translations!

        precision = 0.02

        with Morse() as simu:
            simu.robot.arm.set_rotation("kuka_2", 1).result() # basic rotation
            time.sleep(0.1)
            self.assertAlmostEqual(simu.robot.arm.arm_pose.get()["kuka_2"], 1.0, delta = precision)

            simu.robot.arm.set_rotation("kuka_2", 4000).result() # rotation clamping
            time.sleep(0.1)
            self.assertAlmostEqual(simu.robot.arm.arm_pose.get()["kuka_2"], 2.09, delta = precision)

            res = simu.robot.arm.set_rotation('pipo',0) # inexistant joint
            self.assertEqual(type(res.exception(1)), MorseServiceFailed)

            res = simu.robot.arm.set_translation('kuka_2',0) # non prismatic joint
            self.assertEqual(type(res.exception(1)), MorseServiceFailed)


            # note that set_rotations is tested in armature_pose_testing

    def test_motion_services(self):
        """ Tests the services that have take some time to move
        (joint speed limit taken into account)
        """
        # TODO: check translations!

        precision = 0.02
        with Morse() as simu:
            simu.robot.arm.rotate("kuka_2", 0.5).result() # basic rotation
            self.assertAlmostEqual(simu.robot.arm.arm_pose.get()["kuka_2"], 0.5, delta = precision)
            simu.robot.arm.rotate("kuka_2", 4000).result() # rotation clamping
            self.assertAlmostEqual(simu.robot.arm.arm_pose.get()["kuka_2"], 2.09, delta = precision)

            res = simu.robot.arm.rotate('pipo',0) # inexistant joint
            self.assertEqual(type(res.exception(1)), MorseServiceFailed)

            res = simu.robot.arm.translate('kuka_2',0) # non prismatic joint
            self.assertEqual(type(res.exception(1)), MorseServiceFailed)

            simu.robot.arm.set_rotation("kuka_2", 0).result() # back to origin
            act = simu.robot.arm.rotate("kuka_2", 1, 0.5)
            self.assertFalse(act.done())
            time.sleep(1)
            self.assertFalse(act.done())
            self.assertAlmostEqual(simu.robot.arm.arm_pose.get()["kuka_2"], 0.5, delta = precision)
            time.sleep(1.1)
            self.assertTrue(act.done())

    def test_trajectory(self):

        traj0 = {'points': [
                    {'position': [0.0, 1.57, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'time_from_start': 1.0}
                    ]
                }

        traj1 = {'points': [
                    {'positions': [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'time_from_start': 1.0},
                    {'positions': [0.0, 1.57, 0.0, -1.57, 0.0, 1.57, 0.0],
                     'time_from_start': 4.0},
                    {'positions': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                     'time_from_start': 6.0}
                    ]
                }

        with Morse() as simu:

            act = simu.robot.arm.trajectory(traj0)
            self.assertEqual(type(act.exception(1)), MorseServiceFailed) # typo in trajectory ('position' instead of 'positions')
            self.assertIn('positions', str(act.exception()))

            act = simu.robot.arm.trajectory(traj1)
            time.sleep(1)
            self.checkstate(simu, [0.0, 1.0, 0,0,0,0,0])
            time.sleep(3)
            self.checkstate(simu, [0.0, 1.57, 0, -1.57,0,1.57,0])
            time.sleep(2)
            self.checkstate(simu, [0.0] * 7)
            time.sleep(1)
            self.checkstate(simu, [0.0] * 7)

            # check 'starttime' parameter
            act = simu.robot.arm.set_rotations([0.0] * 7)
            traj2 = {'starttime': time.time() + 1,
                    'points': [
                        {'positions': [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        'time_from_start': 1.0}
                        ]
                    }

            act = simu.robot.arm.trajectory(traj2)
            time.sleep(0.5)
            self.checkstate(simu, [0.0] * 7)
            time.sleep(0.5)
            time.sleep(1)
            self.checkstate(simu, [0.0, 1.0, 0,0,0,0,0])
            time.sleep(1)
            self.checkstate(simu, [0.0, 1.0, 0,0,0,0,0])

            # Check action cancellation
            act = simu.robot.arm.set_rotations([0.0] * 7)

            traj2['starttime'] = time.time() + 1
            act = simu.robot.arm.trajectory(traj2)
            time.sleep(0.5)
            act.cancel()
            time.sleep(1)
            self.checkstate(simu, [0.0] * 7)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(ArmatureTest)
