import logging; logger = logging.getLogger("morse." + __name__)
import json
from morse.middleware.socket_datastream import MorseSocketServ
from functools import partial

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_instance.output_functions.append(partial(MorseSocketServ.main_export, mw_data[-1], function))

def fill_missing_pr2_joints(joints):

    pr2_joints = {'head_pan':0.0, 
                'head_tilt':0.0, 
                'l_shoulder_pan':0.0, 
                'l_shoulder_lift':0.0, 
                'l_upper_arm':0.0, 
                'l_elbow':0.0, 
                'l_forearm':0.0, 
                'l_wrist_flex':0.0, 
                'l_wrist_roll':0.0, 
                'r_shoulder_pan':0.0, 
                'r_shoulder_lift':0.0, 
                'r_upper_arm':0.0, 
                'r_elbow':0.0, 
                'r_forearm':0.0, 
                'r_wrist_flex':0.0, 
                'r_wrist_roll':0.0, 
                'torso_lift_joint':0.0, 
                'laser_tilt_mount_joint':0.0, 
                'fl_caster_rotation_joint':0.0, 
                'fl_caster_l_wheel_joint':0.0, 
                'fl_caster_r_wheel_joint':0.0, 
                'fr_caster_rotation_joint':0.0, 
                'fr_caster_l_wheel_joint':0.0, 
                'fr_caster_r_wheel_joint':0.0, 
                'bl_caster_rotation_joint':0.0, 
                'bl_caster_l_wheel_joint':0.0, 
                'bl_caster_r_wheel_joint':0.0, 
                'br_caster_rotation_joint':0.0, 
                'br_caster_l_wheel_joint':0.0, 
                'br_caster_r_wheel_joint':0.0, 
                'r_gripper_motor_slider_joint':0.0, 
                'r_gripper_motor_screw_joint':0.0, 
                'r_gripper_l_finger_joint':0.0, 
                'r_gripper_r_finger_joint':0.0, 
                'r_gripper_l_finger_tip_joint':0.0, 
                'r_gripper_r_finger_tip_joint':0.0, 
                'r_gripper_joint':0.0, 
                'l_gripper_motor_slider_joint':0.0, 
                'l_gripper_motor_screw_joint':0.0, 
                'l_gripper_l_finger_joint':0.0, 
                'l_gripper_r_finger_joint':0.0, 
                'l_gripper_l_finger_tip_joint':0.0, 
                'l_gripper_r_finger_tip_joint':0.0, 
                'l_gripper_joint':0.0, 
                'torso_lift_motor_screw_joint':0.0, 
                'head_pan_joint':0.0, 
                'head_tilt_joint':0.0, 
                'l_shoulder_pan_joint':0.0, 
                'l_shoulder_lift_joint':0.0, 
                'l_upper_arm_roll_joint':0.0, 
                'l_elbow_flex_joint':0.0, 
                'l_forearm_roll_joint':0.0, 
                'l_wrist_flex_joint':0.0, 
                'l_wrist_roll_joint':0.0, 
                'r_shoulder_pan_joint':0.0, 
                'r_shoulder_lift_joint':0.0, 
                'r_upper_arm_roll_joint':0.0, 
                'r_elbow_flex_joint':0.0, 
                'r_forearm_roll_joint':0.0, 
                'r_wrist_flex_joint':0.0, 
                'r_wrist_roll_joint':0.0}

    for k,v in joints.items():
        for name, value in v.items():
            pr2_joints[name] = value

    return pr2_joints


def post_jointstate(self, component_instance):

    joints = {}

    for k,v in component_instance.local_data.items():
        for name, value in v.items():
            joints[name] = value

    return (json.dumps(joints) + '\n').encode()

def post_pr2_jointstate(self, component_instance):

    joints =  fill_missing_pr2_joints(component_instance.local_data)

    return (json.dumps(joints) + '\n').encode()
