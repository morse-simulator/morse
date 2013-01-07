#! /usr/bin/env python
"""
This script tests the KUKA LWR arm, both the data and service api
"""

import sys
import math
from time import sleep
from morse.testing.testing import MorseTestCase
from pymorse import Morse, MorseServiceFailed

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

        robot = ATRV('robot')

        kuka_lwr = KukaLWR('arm')
        robot.append(kuka_lwr)
        kuka_lwr.translate(z=0.9)
        kuka_lwr.configure_mw('socket')
        kuka_lwr.configure_service('socket')

        pose = Pose("pose")
        pose.configure_mw('socket')
        pose.translate(z=1.3)
        kuka_lwr.append(pose)

        arm_pose = KukaPosture('arm_pose')
        arm_pose.properties(armature = kuka_lwr.name)
        robot.append(arm_pose)
        arm_pose.configure_mw('socket')

        motion = Teleport('motion')
        robot.append(motion)
        motion.configure_service('socket')

        env = Environment('empty', fastmode = True)
        env.configure_service('socket')

    def _test_object_attach(self):
        """ Checks that attached object are indeed attached at the right place.
        """
        precision = 0.02

        with Morse() as simu:

            self.assertAlmostEqual(simu.robot.pose.get()['z'], 2.3, delta = 0.01)
            self.assertAlmostEqual(simu.robot.pose.get()['x'], 0.0, delta = 0.01)
            self.assertAlmostEqual(simu.robot.pose.get()['y'], 0.0, delta = 0.01)
            self.assertAlmostEqual(simu.robot.pose.get()['pitch'], 0.0, delta = 0.01)
            simu.robot.motion.translate(1.0)
            sleep(0.1)
            self.assertAlmostEqual(simu.robot.pose.get()['z'], 2.3, delta = 0.01)
            self.assertAlmostEqual(simu.robot.pose.get()['x'], 1.0, delta = 0.01)
            self.assertAlmostEqual(simu.robot.pose.get()['y'], 0.0, delta = 0.01)
            self.assertAlmostEqual(simu.robot.pose.get()['pitch'], 0.0, delta = 0.01)
            simu.robot.arm.set_rotation("kuka_2", math.radians(-90)).result()
            sleep(0.1)
            self.assertAlmostEqual(simu.robot.pose.get()['z'], 1.31, delta = 0.01)
            self.assertAlmostEqual(simu.robot.pose.get()['x'], 1.99, delta = 0.01)
            self.assertAlmostEqual(simu.robot.pose.get()['pitch'], math.radians(90), delta = 0.01)

    def test_immediate_api(self):
        """ Tests the services that have an immediate result
        (no speed limit taken into account)

        """
        # TODO: check translations!

        precision = 0.02

        with Morse() as simu:
            simu.robot.arm.set_rotation("kuka_2", 1).result() # basic rotation
            sleep(0.1)
            self.assertAlmostEqual(simu.robot.arm_pose.get()["kuka_2"], 1.0, delta = precision)

            simu.robot.arm.set_rotation("kuka_2", 4000).result() # rotation clamping
            sleep(0.1)
            self.assertAlmostEqual(simu.robot.arm_pose.get()["kuka_2"], 2.09, delta = precision)

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
            self.assertAlmostEqual(simu.robot.arm_pose.get()["kuka_2"], 0.5, delta = precision)
            simu.robot.arm.rotate("kuka_2", 4000).result() # rotation clamping
            self.assertAlmostEqual(simu.robot.arm_pose.get()["kuka_2"], 2.09, delta = precision)

            res = simu.robot.arm.rotate('pipo',0) # inexistant joint
            self.assertEqual(type(res.exception(1)), MorseServiceFailed)

            res = simu.robot.arm.translate('kuka_2',0) # non prismatic joint
            self.assertEqual(type(res.exception(1)), MorseServiceFailed)

            simu.robot.arm.set_rotation("kuka_2", 0).result() # back to origin
            act = simu.robot.arm.rotate("kuka_2", 1, 0.5)
            self.assertFalse(act.done())
            sleep(1)
            self.assertFalse(act.done())
            self.assertAlmostEqual(simu.robot.arm_pose.get()["kuka_2"], 0.5, delta = precision)
            sleep(1.1)
            self.assertTrue(act.done())


########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(ArmatureActuatorTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

