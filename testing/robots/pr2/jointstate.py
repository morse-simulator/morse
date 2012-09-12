#! /usr/bin/env python
"""
This script tests ROS services within MORSE.
"""

from morse.testing.testing import MorseTestCase

# Include this import to be able to use your test file as a regular 
# builder script, ie, usable with: 'morse [run|exec] base_testing.py
try:
    from morse.builder import *
except ImportError:
    pass

import os
import sys

try:
    os.environ['MORSE_SRC_ROOT']
except KeyError:
    print("You must define the environment variable MORSE_SRC_ROOT"
          " to point to the MORSE source before running ROS tests.")
    sys.exit(1)

os.environ['ROS_PACKAGE_PATH'] += ":" + os.path.dirname(
        os.path.join(os.environ['MORSE_SRC_ROOT'], "testing", "middlewares", "ros"))

import roslib; roslib.load_manifest("morsetesting")
import rospy
import time
from morsetesting.msg import *
from sensor_msgs.msg import JointState

class PR2JointStateTest(MorseTestCase):

    pr2_joints_list = ['head_pan', 'head_tilt', 'l_shoulder_pan', 'l_shoulder_lift', 'l_upper_arm', 'l_elbow', 'l_forearm', 'l_wrist_flex', 'l_wrist_roll', 'r_shoulder_pan', 'r_shoulder_lift', 'r_upper_arm', 'r_elbow', 'r_forearm', 'r_wrist_flex', 'r_wrist_roll', 'torso_lift_joint', 'laser_tilt_mount_joint', 'fl_caster_rotation_joint', 'fl_caster_l_wheel_joint', 'fl_caster_r_wheel_joint', 'fr_caster_rotation_joint', 'fr_caster_l_wheel_joint', 'fr_caster_r_wheel_joint', 'bl_caster_rotation_joint', 'bl_caster_l_wheel_joint', 'bl_caster_r_wheel_joint', 'br_caster_rotation_joint', 'br_caster_l_wheel_joint', 'br_caster_r_wheel_joint', 'r_gripper_motor_slider_joint', 'r_gripper_motor_screw_joint', 'r_gripper_l_finger_joint', 'r_gripper_r_finger_joint', 'r_gripper_l_finger_tip_joint', 'r_gripper_r_finger_tip_joint', 'r_gripper_joint', 'l_gripper_motor_slider_joint', 'l_gripper_motor_screw_joint', 'l_gripper_l_finger_joint', 'l_gripper_r_finger_joint', 'l_gripper_l_finger_tip_joint', 'l_gripper_r_finger_tip_joint', 'l_gripper_joint', 'torso_lift_motor_screw_joint', 'head_pan_joint', 'head_tilt_joint', 'l_shoulder_pan_joint', 'l_shoulder_lift_joint', 'l_upper_arm_roll_joint', 'l_elbow_flex_joint', 'l_forearm_roll_joint', 'l_wrist_flex_joint', 'l_wrist_roll_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
    
    def setUpEnv(self):
        from morse.builder.extensions.pr2extension import PR2
        print("Adding a PR2 robot...")
        pr2 = PR2()
        pr2_posture = Sensor("pr2_posture")
        pr2.append(pr2_posture)
        pr2_posture.configure_mw('ros')

        env = Environment('indoors-1/indoor-1')
        env.aim_camera([1.0470, 0, 0.7854])
    
    
    def test_jointstates(self):
        rospy.loginfo("Creating listener node to check if posture of PR2 is published.")
        rospy.init_node('pr2_jointstate_listener', log_level = rospy.DEBUG, disable_signals=True)
        rospy.loginfo("Subscribing to pr2_posture topic.")
        #rospy.Subscriber("pr2/pr2_posture", JointState, self.callback)
        jointstate_msg = rospy.client.wait_for_message("pr2/pr2_posture", JointState)
        name_len = len(jointstate_msg.name)
        pos_len = len(jointstate_msg.position)
        rospy.loginfo("Checking if number of jointstate names equals numer of jointstate positions.")
        self.assertEqual(pos_len, name_len, 'Found %s jointstate names but %s jointstate positions. Please check the configuration of your pr2_posture sensor and middleware!'%(name_len, pos_len))
        rospy.loginfo("Checking is every jointstate is present.")
        self.assertEqual(set(self.pr2_joints_list), set(jointstate_msg.name), 'Could not find all joints of the PR2. Please check if you named the joints correctly in your pr2_posture sensor and middleware!' )

########################## Run these tests ##########################
if __name__ == "__main__":
    import unittest
    from morse.testing.testing import MorseTestRunner
    suite = unittest.TestLoader().loadTestsFromTestCase(PR2JointStateTest)
    sys.exit(not MorseTestRunner().run(suite).wasSuccessful())

