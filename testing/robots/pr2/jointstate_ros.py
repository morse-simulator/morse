#! /usr/bin/env python
"""
This script tests ROS services within MORSE.
"""

from morse.testing.ros import RosTestCase

# Include this import to be able to use your test file as a regular
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

import os
import sys

import rospy
import time
from sensor_msgs.msg import JointState

class PR2JointStateTest(RosTestCase):

    # from the output of "rostopic echo /joint_states" on the PR2
    pr2_joints_list = ['fl_caster_rotation_joint', 'fl_caster_l_wheel_joint', 'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint', 'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint', 'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint', 'br_caster_rotation_joint', 'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint', 'torso_lift_joint', 'torso_lift_motor_screw_joint', 'head_pan_joint', 'head_tilt_joint', 'laser_tilt_mount_joint', 'r_upper_arm_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_forearm_roll_joint', 'r_elbow_flex_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint', 'r_gripper_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint', 'r_gripper_r_finger_tip_joint', 'r_gripper_l_finger_tip_joint', 'r_gripper_motor_screw_joint', 'r_gripper_motor_slider_joint', 'l_upper_arm_roll_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_forearm_roll_joint', 'l_elbow_flex_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'l_gripper_joint', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_motor_screw_joint', 'l_gripper_motor_slider_joint']

    def setUpEnv(self):
        print("Adding a PR2 robot...")
        pr2 = BasePR2()
        pr2.add_interface("ros")

        env = Environment('empty', fastmode = True)
        env.set_camera_rotation([1.0470, 0, 0.7854])


    def test_base_jointstates(self):
        rospy.loginfo("Creating listener node to check if posture of PR2 is published.")
        rospy.init_node('pr2_jointstate_listener', disable_signals=True)
        rospy.loginfo("Subscribing to pr2_posture topic.")
        jointstate_msg = rospy.client.wait_for_message("joint_states", JointState)
        name_len = len(jointstate_msg.name)
        pos_len = len(jointstate_msg.position)
        rospy.loginfo("Checking if number of jointstate names equals number of jointstate positions.")
        self.assertEqual(pos_len, name_len, 'Found %s jointstate names but %s jointstate positions. Please check the configuration of your pr2_posture sensor and middleware!'%(name_len, pos_len))
        rospy.loginfo("Checking is every jointstate is present.")
        self.assertEqual(set(self.pr2_joints_list), set(jointstate_msg.name), 'Could not find all joints of the PR2. Please check if you named the joints correctly in your pr2_posture sensor and middleware!' )

########################## Run these tests ##########################
if __name__ == "__main__":
    from morse.testing.testing import main
    main(PR2JointStateTest)
