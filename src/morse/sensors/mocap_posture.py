import logging; logger = logging.getLogger("morse." + __name__)
import math
import bge
import morse.core.sensor

#logger.setLevel(logging.DEBUG)

class HumanPostureClass(morse.core.sensor.MorseSensorClass):
    """ Class definition for the human posture exporter.
        This sensor exports the posture of a human model moving in the simulator.

        A human posture is a vector of 39 floats. The 3 first are the [x,y,z]
        world position, the 3 next are the global rotation on the [x,y,z] axis.
        The 33 other one are described on a separate document (ask easisbot@laas.fr)

        Note: all angles are expressed in radians

        """

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['x'] = 0.0
        self.local_data['y'] = 0.0
        self.local_data['z'] = 0.0
        self.local_data['roll'] = 0.0
        self.local_data['pitch'] = 0.0
        self.local_data['yaw'] = 0.0
        self.local_data['empty1'] = 0.0
        self.local_data['empty2'] = 0.0
        self.local_data['empty3'] = 0.0
        self.local_data['empty4'] = 0.0
        self.local_data['empty5'] = 0.0
        self.local_data['empty6'] = 0.0
        self.local_data['dof_12'] = 0.0     # Torso X
        self.local_data['dof_13'] = 0.0     # Torso Y
        self.local_data['dof_14'] = 0.0     # Torso Z
        self.local_data['dof_15'] = 0.0     # Head Z
        self.local_data['dof_16'] = 0.0     # Head Y
        self.local_data['dof_17'] = 0.0     # Head X
        self.local_data['dof_18'] = 0.0     # R_Shoulder X
        self.local_data['dof_19'] = 0.0     # R_Shoulder Z
        self.local_data['dof_20'] = 0.0     # R_Shoulder Y
        self.local_data['dof_21'] = 0.0     # R_Arm_Trans
        self.local_data['dof_22'] = 0.0     # R_Elbow_Z
        self.local_data['dof_23'] = 0.0     # L_Point
        self.local_data['dof_24'] = 0.0     # R_Wrist X
        self.local_data['dof_25'] = 0.0     # R_Wrist Y
        self.local_data['dof_26'] = 0.0     # R_Wrist Z
        self.local_data['dof_27'] = 0.0     # L_Shoulder X
        self.local_data['dof_28'] = 0.0     # L_Shoulder Z
        self.local_data['dof_29'] = 0.0     # L_Shoulder Y
        self.local_data['dof_30'] = 0.0     # L_Arm_Trans
        self.local_data['dof_31'] = 0.0     # L_Elbow_Z
        self.local_data['dof_32'] = 0.0     # L_Point
        self.local_data['dof_33'] = 0.0     # L_Wrist X
        self.local_data['dof_34'] = 0.0     # L_Wrist Y
        self.local_data['dof_35'] = 0.0     # L_Wrist Z
        self.local_data['dof_36'] = 0.0     # R_Hip X
        self.local_data['dof_37'] = 0.0     # R_Hip Y
        self.local_data['dof_38'] = 0.0     # R_Hip Z
        self.local_data['dof_39'] = 0.0     # R_Knee
        self.local_data['dof_40'] = 0.0     # R_Ankle X
        self.local_data['dof_41'] = 0.0     # R_Ankle Y
        self.local_data['dof_42'] = 0.0     # R_Ankle Z
        self.local_data['dof_43'] = 0.0     # L_Hip X
        self.local_data['dof_44'] = 0.0     # L_Hip Y
        self.local_data['dof_45'] = 0.0     # L_Hip Z
        self.local_data['dof_46'] = 0.0     # L_Knee
        self.local_data['dof_47'] = 0.0     # L_Ankle X
        self.local_data['dof_48'] = 0.0     # L_Ankle Y
        self.local_data['dof_49'] = 0.0     # L_Ankle Z

        """
        self.data_keys = self.local_data.keys()

        # Initialise the copy of the data
        for variable in self.data_keys:
            self.modified_data.append(self.local_data[variable])
        """

        logger.info('Component initialized')

    def _read_pose(self, armature):

        for channel in armature.channels:
            if 'X_' not in channel.name:
                #logger.debug("\tChannel '%s': (%.4f, %.4f, %.4f)" % (channel, channel.joint_rotation[0], channel.joint_rotation[1], channel.joint_rotation[2]))
                if channel.name == 'Chest':
                    self.local_data['dof_12'] = channel.joint_rotation[0]
                    self.local_data['dof_13'] = channel.joint_rotation[2]
                    self.local_data['dof_14'] = - channel.joint_rotation[1]
                if channel.name == 'Head':
                    self.local_data['dof_15'] = - channel.joint_rotation[0] #z axis
                    self.local_data['dof_16'] = channel.joint_rotation[2] #x axis
                    self.local_data['dof_17'] = channel.joint_rotation[1] #y axis

                if channel.name == 'UpArm.R':
                    self.local_data['dof_18'] = - channel.joint_rotation[0]
                    self.local_data['dof_19'] = channel.joint_rotation[2]
                if channel.name == 'ForeArm.R':
                    self.local_data['dof_20'] = -channel.joint_rotation[2]
                    self.local_data['dof_22'] = channel.joint_rotation[0]
                # Kinect does not provide rotations for the hands
                #if channel.name == 'Hand.R':
                #    self.local_data['dof_24'] = channel.joint_rotation[0]
                #    self.local_data['dof_25'] = channel.joint_rotation[1]
                #    self.local_data['dof_26'] = channel.joint_rotation[2]

                if channel.name == 'UpArm.L':
                    self.local_data['dof_27'] = - channel.joint_rotation[0]
                    self.local_data['dof_28'] = - channel.joint_rotation[2]
                if channel.name == 'ForeArm.L':
                    self.local_data['dof_29'] = -channel.joint_rotation[2]
                    self.local_data['dof_31'] = channel.joint_rotation[0]
                # Kinect does not provide rotations for the hands
                #if channel.name == 'Hand.L':
                #    self.local_data['dof_33'] = channel.joint_rotation[0]
                #    self.local_data['dof_34'] = channel.joint_rotation[1]
                #    self.local_data['dof_35'] = channel.joint_rotation[2]

                
                if channel.name == 'UpLeg.R' :
                    self.local_data['dof_36'] = - channel.joint_rotation[0]
                    self.local_data['dof_37'] = - channel.joint_rotation[2]
                    self.local_data['dof_38'] = - channel.joint_rotation[1]
                if channel.name == 'LoLeg.R':
                    self.local_data['dof_39'] = - channel.joint_rotation[2]
                # Kinect does not provide rotations for the feet
                #if channel.name == 'Foot.R':
                #    self.local_data['dof_40'] = channel.joint_rotation[0]
                #    self.local_data['dof_41'] = channel.joint_rotation[1]
                #    self.local_data['dof_42'] = channel.joint_rotation[2]

                if channel.name == 'UpLeg.L':
                    self.local_data['dof_43'] = - channel.joint_rotation[0]
                    self.local_data['dof_44'] = - channel.joint_rotation[2]
                    self.local_data['dof_45'] = - channel.joint_rotation[1]
                if channel.name == 'LoLeg.L':
                    #print ("%s = [%.2f, %.2f, %.2f]" % (channel.name, channel.joint_rotation[0],channel.joint_rotation[1],channel.joint_rotation[2]))
                    self.local_data['dof_46'] = - channel.joint_rotation[2]
                # Kinect does not provide rotations for the feet
                #if channel.name == 'Foot.L':
                #    self.local_data['dof_47'] = channel.joint_rotation[0]
                #    self.local_data['dof_48'] = channel.joint_rotation[1]
                #    self.local_data['dof_49'] = channel.joint_rotation[2]



    def default_action(self):
        """ Extract the human posture """
        #logger.debug("\tI am '%s': (%.4f, %.4f, %.4f)" % (self.name, self.position_3d.x, self.position_3d.y, self.position_3d.z))
        #self.local_data['x'] = float(self.position_3d.x)
        #self.local_data['y'] = float(self.position_3d.y)
        #self.local_data['z'] = float(self.position_3d.z)

        #self.local_data['yaw'] = float(self.position_3d.yaw)# - (math.pi/2) # 1.57 #pi/2
        #self.local_data['pitch'] = float(self.position_3d.pitch)
        #self.local_data['roll'] = float(self.position_3d.roll)

        # Give the position of the Torso_Empty object as the position of the human
        scene = bge.logic.getCurrentScene()
        torso = scene.objects['Torso_Empty']
        self.local_data['x'] = torso.worldPosition[0]
        self.local_data['y'] = torso.worldPosition[1]
        self.local_data['z'] = torso.worldPosition[2]
        logger.debug("\tTorso_Empty position: (%.4f, %.4f, %.4f)" % (torso.worldPosition[0], torso.worldPosition[1], torso.worldPosition[2]))

        # Pass also the rotation of the Torso_Empty
        self.local_data['yaw'] = torso.worldOrientation.to_euler().z
        self.local_data['pitch'] = torso.worldOrientation.to_euler().y
        self.local_data['roll'] = torso.worldOrientation.to_euler().x
        logger.debug("\tTorso_Empty orientation: (%.4f, %.4f, %.4f)" % (self.local_data['roll'], self.local_data['pitch'], self.local_data['yaw']))

        self._read_pose(self.blender_obj)

        logger.debug("LOCAL_DATA: %s" % self.local_data)
