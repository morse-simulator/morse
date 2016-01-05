#! /usr/bin/env python
"""
This script tests the inverse kinematics of the human model.
"""

from morse.testing.testing import MorseTestCase
from pymorse import Morse

# Include this import to be able to use your test file as a regular
# builder script, ie, usable with: 'morse [run|exec] <your test>.py
try:
    from morse.builder import *
except ImportError:
    pass

class HumanIkTest(MorseTestCase):

    TESTED_JOINTS = ["neck03","lowerleg01_L","lowerleg01_R","upperarm02_L","upperarm02_R"]

    def _print_ik_joints_state(self, skeleton):
        print("---")
        for j in self.TESTED_JOINTS:
            print("%s: %s" % (j, skeleton.joint_states.get_state().result()[j]))
        print("---")

    def _ik_joint(self, skeleton, name):
        return skeleton.joint_states.get_state().result()[name.split(".")[-1]]

    def _check_pose(self, skeleton, neck, leg_l, leg_r, arm_l, arm_r):
        self.assertAlmostEqual(self._ik_joint(skeleton, self.TESTED_JOINTS[0]), neck, delta = 0.05)
        self.assertAlmostEqual(self._ik_joint(skeleton, self.TESTED_JOINTS[1]), leg_l, delta = 0.05)
        self.assertAlmostEqual(self._ik_joint(skeleton, self.TESTED_JOINTS[2]), leg_r, delta = 0.05)
        self.assertAlmostEqual(self._ik_joint(skeleton, self.TESTED_JOINTS[3]), arm_l, delta = 0.05)
        self.assertAlmostEqual(self._ik_joint(skeleton, self.TESTED_JOINTS[4]), arm_r, delta = 0.05)

    def setUpEnv(self):
        """ Defines the test scenario, using the Builder API.
        """

        human = Human()
        human.add_interface('socket')

        env = Environment('empty', fastmode=True)
        env.set_camera_location([2.6,-0.3,1.2])
        env.set_camera_rotation([1.3,0,1.4])
        env.add_service('socket')

    def test_ik_motion(self):

        IK_TARGET_HEAD = "ik_target.human.skeleton.head"
        IK_TARGET_FOOT_L = "ik_target.human.skeleton.foot_L"
        IK_TARGET_FOOT_R = "ik_target.human.skeleton.foot_R"
        IK_TARGET_WRIST_L = "ik_target.human.skeleton.wrist_L"
        IK_TARGET_WRIST_R = "ik_target.human.skeleton.wrist_R"
        IK_TARGETS = set((IK_TARGET_HEAD, IK_TARGET_FOOT_L, IK_TARGET_FOOT_R, IK_TARGET_WRIST_L, IK_TARGET_WRIST_R))

        with Morse() as morse:
            skeleton = morse.human.skeleton

            self.assertEqual(set(skeleton.list_IK_targets().result()), IK_TARGETS)

            # Set the human in a resting position
            skeleton.place_IK_target(IK_TARGET_HEAD, [0.04,0.,1.58], [1.57,0.05,1.58], False)
            skeleton.place_IK_target(IK_TARGET_FOOT_L, [0.02,0.10,0.07], [-2.65,-0.32,1.43], False)
            skeleton.place_IK_target(IK_TARGET_FOOT_R, [0.02,-0.13,0.07], [-2.65,0.32,1.70], False)
            skeleton.place_IK_target(IK_TARGET_WRIST_L, [0.03,0.29,0.87], [-1.97,0.,1.47], False)
            skeleton.place_IK_target(IK_TARGET_WRIST_R, [0.05,-0.25,0.88], [1.33,2.91,-1.11], False)

            morse.sleep(1)
            self._check_pose(skeleton, -0.006, -0.016, -0.013, 0., 0.01)

            # left hand, palm facing up
            skeleton.move_IK_target(IK_TARGET_WRIST_L, [0.44,0.31,1.1], [-3,1.6,1.5], False).result()
            self._check_pose(skeleton, -0.087, -0.016, -0.013, 0.33, -0.01)

            # back to rest pose
            skeleton.move_IK_target(IK_TARGET_WRIST_L, [0.03,0.29,0.87], [-1.97,0.,1.47], False).result()
            self._check_pose(skeleton, -0.006, -0.016, -0.013, 0., 0.01)

            # right hand above head
            skeleton.move_IK_target(IK_TARGET_WRIST_R, [0.04,-0.15,1.76], [-1.41,1.8,-1.33], False).result()
            self._check_pose(skeleton, 0.35, -0.011,-0.013, -0.059, 0.26)

            # back to rest pose
            skeleton.move_IK_target(IK_TARGET_WRIST_R, [0.05,-0.25,0.88], [1.33,2.91,-1.11], False).result()
            self._check_pose(skeleton, -0.006, -0.016, -0.013, 0., 0.01)

            # head looking left (relative motion)
            skeleton.move_IK_target(IK_TARGET_HEAD, [0.,0.,0.], [0.,0.,1.], True).result()
            self._check_pose(skeleton, 0.35, -0.016, -0.013, 0.01, 0.02)

            # head looking right (relative motion)
            skeleton.move_IK_target(IK_TARGET_HEAD, [0.,0.,0.], [0.,0.,-2.], True).result()
            self._check_pose(skeleton, -0.34, -0.016, -0.013, -0.01, 0.02)


########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(HumanIkTest)
