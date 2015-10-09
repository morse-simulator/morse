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

JOINTS = ['kuka_1', 'kuka_2', 'kuka_3', 'kuka_4', 'kuka_5', 'kuka_6', 'kuka_7']

class ArmatureTest(MorseTestCase):
    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        robot = FakeRobot()

        arm = KukaLWR()
        robot.append(arm)
        arm.add_stream('socket')
        arm.add_service('socket')

        pose = Pose()
        pose.add_stream('socket')
        pose.translate(z=1.3105)
        arm.append(pose)

        arm_pose = ArmaturePose()
        arm.append(arm_pose)
        arm_pose.add_stream('socket')

        motion = Teleport()
        robot.append(motion)
        motion.add_service('socket')

        env = Environment('empty', fastmode = True)
        env.add_service('socket')

    def _check_state(self, simu, angles, precision = 0.050):

        pose = simu.robot.arm.arm_pose.get()
        target = dict(zip(JOINTS, angles))

        for j, v in pose.items():
            if j != 'timestamp':
                self.assertAlmostEqual(v, target[j], delta=precision)

    def _check_pose(self, simu, x, y, z, pitch, precision = 0.01):
            self.assertAlmostEqual(simu.robot.arm.pose.get()['x'], x, delta = precision)
            self.assertAlmostEqual(simu.robot.arm.pose.get()['y'], y, delta = precision)
            self.assertAlmostEqual(simu.robot.arm.pose.get()['z'], z, delta = precision)
            self.assertAlmostEqual(simu.robot.arm.pose.get()['pitch'], pitch, delta = precision)


    def test_joints_names(self):
        with Morse() as simu:
            self.assertEquals(simu.robot.arm.get_joints(), JOINTS)

    def test_object_attach(self):
        """ Checks that attached object are indeed attached at the right place.
        """
        precision = 0.02

        with Morse() as simu:

            self._check_pose(simu, 0., 0., 1.3105, 0.)
            simu.robot.motion.translate(1.0)
            simu.sleep(0.1)
            self._check_pose(simu, 1., 0., 1.3105, 0.)
            simu.robot.arm.set_rotation("kuka_2", math.radians(-90)).result()
            simu.sleep(0.1)
            self._check_pose(simu, 2., 0., 0.3105, math.radians(90))

    def test_immediate_api(self):
        """ Tests the services that have an immediate result
        (no speed limit taken into account)

        """
        # TODO: check translations!

        precision = 0.02

        with Morse() as simu:
            simu.robot.arm.set_rotation("kuka_2", 1).result() # basic rotation
            simu.sleep(0.1)
            self.assertAlmostEqual(simu.robot.arm.arm_pose.get()["kuka_2"], 1.0, delta = precision)

            simu.robot.arm.set_rotation("kuka_2", 4000).result() # rotation clamping
            simu.sleep(0.1)
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
            simu.sleep(1)
            self.assertFalse(act.done())
            self.assertAlmostEqual(simu.robot.arm.arm_pose.get()["kuka_2"], 0.5, delta = precision)
            simu.sleep(1.1)
            self.assertTrue(act.done())

            kuka_4 = simu.robot.arm.arm_pose.get()["kuka_4"]
            simu.robot.arm.rotate_joints({"kuka_2": 0.5, "kuka_3" : 0.2}).result()
            self.assertAlmostEqual(simu.robot.arm.arm_pose.get()["kuka_2"], 0.5, delta = precision)
            self.assertAlmostEqual(simu.robot.arm.arm_pose.get()["kuka_3"], 0.2, delta = precision)
            self.assertAlmostEqual(simu.robot.arm.arm_pose.get()["kuka_4"], kuka_4, delta = precision)

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
            simu.sleep(1)
            self._check_state(simu, [0.0, 1.0, 0,0,0,0,0])
            simu.sleep(3)
            self._check_state(simu, [0.0, 1.57, 0, -1.57, 0, 1.57, 0])
            simu.sleep(2)
            self._check_state(simu, [0.0] * 7)
            simu.sleep(1)
            self._check_state(simu, [0.0] * 7)

            # check 'starttime' parameter
            act = simu.robot.arm.set_rotations([0.0] * 7)
            traj2 = {'starttime': simu.time() + 1,
                    'points': [
                        {'positions': [0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                        'time_from_start': 1.0}
                        ]
                    }

            act = simu.robot.arm.trajectory(traj2)
            simu.sleep(0.5)
            self._check_state(simu, [0.0] * 7)
            simu.sleep(0.5)
            simu.sleep(1)
            self._check_state(simu, [0.0, 1.0, 0,0,0,0,0])
            simu.sleep(1)
            self._check_state(simu, [0.0, 1.0, 0,0,0,0,0])

            # Check action cancellation
            act = simu.robot.arm.set_rotations([0.0] * 7)

            traj2['starttime'] = simu.time() + 1
            act = simu.robot.arm.trajectory(traj2)
            simu.sleep(0.5)
            act.cancel()
            simu.sleep(1)
            self._check_state(simu, [0.0] * 7)

    def test_ik_immediate(self):

        IK_TARGET = "ik_target.robot.arm.kuka_7"

        with Morse() as simu:
            self.assertEqual(simu.robot.arm.list_IK_targets(), [IK_TARGET])
            self._check_pose(simu, 0., 0., 1.3105, 0.)

            simu.robot.arm.place_IK_target(IK_TARGET, [0,0,2], None, False) # absolute location
            simu.sleep(0.1)
            self._check_pose(simu, 0., 0., 1.3105, 0.)

            simu.robot.arm.place_IK_target(IK_TARGET, [1,0,0.3105], None, False)
            simu.sleep(1) # with iterative IK solvers like iTaSC, the IK chain does not reach the end position immediately
            self._check_pose(simu, 0.778, 0., 0.363, 0.02)

            simu.robot.arm.place_IK_target(IK_TARGET, [1,0,0.3105], [math.pi/2, -math.pi/2, -math.pi], False) # arm should be horizontal
            simu.sleep(1) # with iterative IK solvers like iTaSC, the IK chain does not reach the end position immediately
            self._check_pose(simu, 1.0, 0., 0.3105, math.radians(90))

            # back to original position
            simu.robot.arm.place_IK_target(IK_TARGET, [0,0,2], [math.pi/2, 0., -math.pi], False) # absolute location
            simu.sleep(1.)
            self._check_pose(simu, 0., 0., 1.3105, 0.)

            simu.robot.arm.place_IK_target(IK_TARGET, [-1, 0, -1.6895], None) # relative position
            simu.sleep(1) # with iterative IK solvers like iTaSC, the IK chain does not reach the end position immediately
            self._check_pose(simu, -0.778, 0., 0.363, -0.02)

            simu.robot.arm.place_IK_target(IK_TARGET, [0.,0.,0.], [0., -math.pi/2, 0.]) # relative rotation
            simu.sleep(1) # with iterative IK solvers like iTaSC, the IK chain does not reach the end position immediately
            self._check_pose(simu, -1.0, 0., 0.3105, -math.radians(90))




    def test_ik_motion(self):

        IK_TARGET = "ik_target.robot.arm.kuka_7"

        with Morse() as simu:
            self.assertEqual(simu.robot.arm.list_IK_targets(), [IK_TARGET])
            self._check_pose(simu, 0., 0., 1.3105, 0.)

            simu.robot.arm.move_IK_target(IK_TARGET, [0,0,2], None, False).result() # absolute location
            self._check_pose(simu, 0., 0., 1.3105, 0.)

            simu.robot.arm.move_IK_target(IK_TARGET, [1,0,0.3105], None, False).result()
            self._check_pose(simu, 0.778, 0., 0.363, 0.02)

            simu.robot.arm.move_IK_target(IK_TARGET, [1,0,0.3105], [math.pi/2, -math.pi/2, -math.pi], False).result() # arm should be horizontal
            self._check_pose(simu, 1.0, 0., 0.3105, math.radians(90))

            # back to original position
            simu.robot.arm.move_IK_target(IK_TARGET, [0,0,2], [math.pi/2, 0., -math.pi], False).result() # absolute location
            self._check_pose(simu, 0., 0., 1.3105, 0.)

            simu.robot.arm.move_IK_target(IK_TARGET, [-1, 0, -1.6895], None).result() # relative position
            self._check_pose(simu, -0.778, 0., 0.363, -0.02)

            simu.robot.arm.move_IK_target(IK_TARGET, [0.,0.,0.], [0., -math.pi/2, 0.]).result() # relative rotation
            self._check_pose(simu, -1.0, 0., 0.3105, -math.radians(90))

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(ArmatureTest)
