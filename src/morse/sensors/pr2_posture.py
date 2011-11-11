import logging; logger = logging.getLogger("morse." + __name__)
import math
import morse.core.sensor
import mathutils
import sys
import morse.helpers.math as morse_math

class PR2PostureClass(morse.core.sensor.MorseSensorClass):
    """ Jido posture sensor. Currently working with PTU and KUKA arm """

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

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

        # joints of PTU-unit
        self.local_data['head_pan'] = 0.0
        self.local_data['head_tilt'] = 0.0
        
        ###### The following joints are not updated yet! (TODO)
        
        # laser joint
        self.local_data['laser_tilt_mount_joint'] = 0.0
        
        # wheels (not sure if we also want to simlulate those...)
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

        
        # other joints i don´t know...
        self.local_data['torso_lift_motor_screw_joint'] = 0.0
        self.local_data['laser_tilt_mount_joint'] = 0.0
        self.local_data['laser_tilt_mount_joint'] = 0.0
        self.local_data['laser_tilt_mount_joint'] = 0.0
        
        # passive joints (what are they for?)
        self.local_data['r_gripper_l_finger_joint'] = 0.0
        self.local_data['r_gripper_r_finger_joint'] = 0.0
        self.local_data['r_gripper_r_finger_tip_joint'] = 0.0
        self.local_data['r_gripper_l_finger_tip_joint'] = 0.0
        self.local_data['l_gripper_l_finger_joint'] = 0.0
        self.local_data['l_gripper_r_finger_joint'] = 0.0
        self.local_data['l_gripper_r_finger_tip_joint'] = 0.0
        self.local_data['l_gripper_l_finger_tip_joint'] = 0.0


        
        # Get names of armatures (for ptu and arm joints)
        self.ptu_armature_name = self.robot_parent.armatures[0]
        self.l_arm_armature_name = self.robot_parent.armatures[1]
        self.r_arm_armature_name = self.robot_parent.armatures[2]
        logger.info("Found PTU: %s"%self.ptu_armature_name)
        logger.info("Found Left arm: %s"%self.l_arm_armature_name)
        logger.info("Found Right arm: %s"%self.r_arm_armature_name)
        self.ptu_rotations = {}
        self.l_arm_rotations = {}
        self.r_arm_rotations = {}
        
        # constant that holds the original height of the torso from the ground
        # These values come from the pr2 urdf file
        self.TORSO_BASE_HEIGHT = (0.739675 + 0.051)
        self.TORSO_LOWER = 0.0  # lower limit on the torso z-translantion
        self.TORSO_UPPER = 0.31  # upper limit on the torso z-translation
        logger.info('Component initialized')
        
    def default_action(self):
        """ Collect jointstates and publish them to ROS """
        
        ##################### get joints of PTU and arm-armatures and objects##################
        # Get Blender object of armatures
        for child in self.robot_parent.blender_obj.childrenRecursive:
            if child.name == self.ptu_armature_name:
                self.ptu_armature_object = child
            elif child.name == self.l_arm_armature_name:
                self.l_arm_armature_object = child
            elif child.name == self.r_arm_armature_name :
                self.r_arm_armature_object = child
            elif child.name == 'torso_lift_joint':
                self.torso_object = child
        
        for channel in self.ptu_armature_object.channels:
            self.ptu_rotations[channel.name] = channel.joint_rotation.to_tuple()
        
        for channel in self.l_arm_armature_object.channels:
            self.l_arm_rotations[channel.name] = channel.joint_rotation.to_tuple()
            
        for channel in self.r_arm_armature_object.channels:
            self.r_arm_rotations[channel.name] = channel.joint_rotation.to_tuple()
        
        self.torso = self.torso_object.localPosition[2] - self.TORSO_BASE_HEIGHT
        
        print("PTU Rotations are: %s"%self.ptu_rotations)
        print("L arm Rotations are: %s"%self.l_arm_rotations)
        print("R arm Rotations are: %s"%self.r_arm_rotations)
        print("Torso joint: %s"%self.torso)

        ############################# Hand data over to middleware ##############################

        # Before handing over to middleware, find out what values are postet on real PR2 (full tuples of angles or 
        # only values for specific degree of freedom?)
        
        # KUKA arm
        #self.local_data['seg0'] = self._angles[0]
        #self.local_data['seg1'] = self._angles[1]
        #self.local_data['seg2'] = self._angles[2]
        #self.local_data['seg3'] = self._angles[3]
        #self.local_data['seg4'] = self._angles[4]
        #self.local_data['seg5'] = self._angles[5]
        #self.local_data['seg6'] = self._angles[6]

        # PTU 
        #self.local_data['pan'] = float(current_pan)
        #self.local_data['tilt'] = float(current_tilt)
