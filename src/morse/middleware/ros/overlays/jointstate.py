from morse.middleware.ros_request_manager import ros_action, ros_service
from morse.core.overlay import MorseOverlay
from morse.core import status

class JointState(MorseOverlay):
    """ 
    This overlay reads the full pr2 configuration from the various armatures, and
    returns a global joint state in ROS format.

    It must be applied to the 'torso' armature component.
    """

    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(overlaid_object)

        # Retrieve pointers to the various pose sensors
        self.torso = self._get_child_objects("pr2.torso.pose", overlaid_object)
        self.head = self._get_child_objects("pr2.torso.head.pose", overlaid_object)
        self.l_arm = self._get_child_objects("pr2.torso.l_arm.pose", overlaid_object)
        self.r_arm = self._get_child_objects("pr2.torso.r_arm.pose", overlaid_object)

        # joints of head
        self.local_data['head_pan'] = 0.0
        self.local_data['head_tilt'] = 0.0
        
        # joints of left-arm
        self.local_data['l_shoulder_pan'] = 0.0
        self.local_data['l_shoulder_lift'] = 0.0
        self.local_data['l_upper_arm'] = 0.0
        self.local_data['l_elbow'] = 0.0
        self.local_data['l_forearm'] = 0.0
        self.local_data['l_wrist_flex'] = 0.0
        self.local_data['l_wrist_roll'] = 0.0
        
        # joints of right-arm
        self.local_data['r_shoulder_pan'] = 0.0
        self.local_data['r_shoulder_lift'] = 0.0
        self.local_data['r_upper_arm'] = 0.0
        self.local_data['r_elbow'] = 0.0
        self.local_data['r_forearm'] = 0.0
        self.local_data['r_wrist_flex'] = 0.0
        self.local_data['r_wrist_roll'] = 0.0
        
        # torse lift
        self.local_data['torso_lift_joint'] = 0.0
        
        ###### The following joints are not updated yet! (TODO)
        
        # laser joint
        self.local_data['laser_tilt_mount_joint'] = 0.0
        
        # wheels
        self.local_data['fl_caster_rotation_joint'] = 0.0
        self.local_data['fl_caster_l_wheel_joint'] = 0.0
        self.local_data['fl_caster_r_wheel_joint'] = 0.0
        self.local_data['fr_caster_rotation_joint'] = 0.0
        self.local_data['fr_caster_l_wheel_joint'] = 0.0
        self.local_data['fr_caster_r_wheel_joint'] = 0.0
        self.local_data['bl_caster_rotation_joint'] = 0.0
        self.local_data['bl_caster_l_wheel_joint'] = 0.0
        self.local_data['bl_caster_r_wheel_joint'] = 0.0
        self.local_data['br_caster_rotation_joint'] = 0.0
        self.local_data['br_caster_l_wheel_joint'] = 0.0
        self.local_data['br_caster_r_wheel_joint'] = 0.0
        
        # gripper joints
        self.local_data['r_gripper_motor_slider_joint'] = 0.0
        self.local_data['r_gripper_motor_screw_joint'] = 0.0
        self.local_data['r_gripper_l_finger_joint'] = 0.0
        self.local_data['r_gripper_r_finger_joint'] = 0.0
        self.local_data['r_gripper_l_finger_tip_joint'] = 0.0
        self.local_data['r_gripper_r_finger_tip_joint'] = 0.0
        self.local_data['r_gripper_joint'] = 0.0
        
        self.local_data['l_gripper_motor_slider_joint'] = 0.0
        self.local_data['l_gripper_motor_screw_joint'] = 0.0
        self.local_data['l_gripper_l_finger_joint'] = 0.0
        self.local_data['l_gripper_r_finger_joint'] = 0.0
        self.local_data['l_gripper_l_finger_tip_joint'] = 0.0
        self.local_data['l_gripper_r_finger_tip_joint'] = 0.0
        self.local_data['l_gripper_joint'] = 0.0
        
        # other joints
        self.local_data['torso_lift_motor_screw_joint'] = 0.0
        self.local_data['laser_tilt_mount_joint'] = 0.0
        self.local_data['laser_tilt_mount_joint'] = 0.0
        self.local_data['laser_tilt_mount_joint'] = 0.0
        
        # passive joints
        self.local_data['r_gripper_l_finger_joint'] = 0.0
        self.local_data['r_gripper_r_finger_joint'] = 0.0
        self.local_data['r_gripper_r_finger_tip_joint'] = 0.0
        self.local_data['r_gripper_l_finger_tip_joint'] = 0.0
        self.local_data['l_gripper_l_finger_joint'] = 0.0
        self.local_data['l_gripper_r_finger_joint'] = 0.0
        self.local_data['l_gripper_r_finger_tip_joint'] = 0.0
        self.local_data['l_gripper_l_finger_tip_joint'] = 0.0

    def _get_child_objects(self, name, obj):
        for child in obj.childrenRecursive():
            if child.name == name:
                return child

    def name():
        return "pr2.joint_state"

    def default_action(self):

         # joints of head
        self.local_data['head_pan'] = self.head.get_joint("head_pan")
        self.local_data['head_tilt'] = self.head.get_joint("head_tilt")
        
        # joints of left-arm
        self.local_data['l_shoulder_pan'] = self.l_arm.get_joint("l_shoulder_pan")
        self.local_data['l_shoulder_lift'] = self.l_arm.get_joint("l_shoulder_lift")
        self.local_data['l_upper_arm'] = self.l_arm.get_joint("l_upper_arm")
        self.local_data['l_elbow'] = self.l_arm.get_joint("l_elbow")
        self.local_data['l_forearm'] = self.l_arm.get_joint("l_forearm")
        self.local_data['l_wrist_flex'] = self.l_arm.get_joint("l_wrist_flex")
        self.local_data['l_wrist_roll'] = self.l_arm.get_joint("l_wrist_roll")
        
        # joints of right-arm
        self.local_data['r_shoulder_pan'] = self.l_arm.get_joint("r_shoulder_pan")
        self.local_data['r_shoulder_lift'] = self.l_arm.get_joint("r_shoulder_lift")
        self.local_data['r_upper_arm'] = self.l_arm.get_joint("r_upper_arm")
        self.local_data['r_elbow'] = self.l_arm.get_joint("r_elbow")
        self.local_data['r_forearm'] = self.l_arm.get_joint("r_forearm")
        self.local_data['r_wrist_flex'] = self.l_arm.get_joint("r_wrist_flex")
        self.local_data['r_wrist_roll'] = self.l_arm.get_joint("r_wrist_roll")
        
        # torse lift
        self.local_data['torso_lift_joint'] = self.torso.get_joint("torso_lift_joint")
