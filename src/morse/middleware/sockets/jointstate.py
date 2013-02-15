import logging; logger = logging.getLogger("morse." + __name__)
import json
from morse.middleware.socket_datastream import SocketPublisher

def fill_missing_pr2_joints(joints):

    pr2_joints = {
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
                'torso_lift_joint':0.0, 
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

class PR2JointStatePublisher(SocketPublisher):

    _type_name = "a JSON dict containing the values of each of the Willow Garage's PR2 joints"

    def encode(self):
        joints =  fill_missing_pr2_joints(self.component_instance.local_data)
        return (json.dumps(joints) + '\n').encode()
