import logging; logger = logging.getLogger("morse." + __name__)
import math
import morse.core.sensor
from morse.helpers.components import add_data

class HumanPosture(morse.core.sensor.Sensor):
    """ 
    This sensor collects the positions of the bones in the human armature
    for the file ``$MORSE_ROOT/data/robots/human.blend``.

    It stores the position and orientation of the general armature
    object, as well as the local rotation of each individual bone. The
    rotation angles are given in radians.

    This sensor will only work for the ``human.blend`` model, as it
    uses a specific naming convention for each of the bones.

    You can also check to general documentation of the :doc:`human
    component <../others/human>`.

    .. image:: ../../../media/human_joints.png
      :align: center
      :width: 600
    """

    _name = "Human Posture"

    add_data('x', 0.0, "float",
             'global X position of the armature in the scene, in meter')
    add_data('y', 0.0, "float",
             'global Y position of the armature in the scene, in meter')
    add_data('z', 0.0, "float",
             'global Z position of the armature in the scene, in meter')
    add_data('yaw', 0.0, "float",
             'rotation angle with respect to the Z axis, in radian')
    add_data('pitch', 0.0, "float",
             'rotation angle with respect to the Z axis, in radian')
    add_data('roll', 0.0, "float",
             'rotation angle with respect to the Z axis, in radian')
    add_data('empty1', 0.0, "float")
    add_data('empty2', 0.0, "float")
    add_data('empty3', 0.0, "float")
    add_data('empty4', 0.0, "float")
    add_data('empty5', 0.0, "float")
    add_data('empty6', 0.0, "float")

    add_data('dof_12', 0.0, 'float',
             'rotation around the X axis for the torso, in radian')
    add_data('dof_13', 0.0, 'float',
             'rotation around the Y axis for the torso, in radian')
    add_data('dof_14', 0.0, 'float',
             'rotation around the Z axis for the torso, in radian')

    add_data('dof_15', 0.0, 'float',
             'rotation around the Z axis for the head, in radian')
    add_data('dof_16', 0.0, 'float',
             'rotation around the Y axis for the head, in radian')
    add_data('dof_17', 0.0, 'float',
             'rotation around the X axis for the head, in radian')

    add_data('dof_18', 0.0, 'float',
             'rotation around the X axis for the right shoulder, in radian')
    add_data('dof_19', 0.0, 'float',
             'rotation around the Y axis for the right shoulder, in radian')
    add_data('dof_20', 0.0, 'float',
             'rotation around the Z axis for the right shoulder, in radian')

    add_data('dof_21', 0.0, 'float',
             'elongation of the right arm, in meter')

    add_data('dof_22', 0.0, 'float',
            'rotation around the Z axis for the right elbow, in radian')

    add_data('dof_23', 0.0, 'float', 'R_POINT')

    add_data('dof_24', 0.0, 'float',
             'rotation around the X axis for the right wrist, in radian')
    add_data('dof_25', 0.0, 'float',
             'rotation around the Y axis for the right wrist, in radian')
    add_data('dof_26', 0.0, 'float',
             'rotation around the Z axis for the right write, in radian')

    add_data('dof_27', 0.0, 'float',
            'rotation around the X axis for the left shoulder, in radian')
    add_data('dof_28', 0.0, 'float',
            'rotation around the Y axis for the left shoulder, in radian')
    add_data('dof_29', 0.0, 'float',
            'rotation around the Z axis for the left shoulder, in radian')

    add_data('dof_30', 0.0, 'float',
             'elongation of the left arm, in meter')

    add_data('dof_31', 0.0, 'float',
             'rotation around the Z axis for the left elbow, in radian')

    add_data('dof_32', 0.0, 'float', 'L_POINT')

    add_data('dof_33', 0.0, 'float', 
            'rotation around the X axis for the left wrist, in radian')
    add_data('dof_34', 0.0, 'float', 
            'rotation around the Y axis for the left wrist, in radian')
    add_data('dof_35', 0.0, 'float', 
            'rotation around the Z axis for the left wrist, in radian')

    add_data('dof_36', 0.0, 'float', 
            'rotation around the X axis for the right hip, in radian')
    add_data('dof_37', 0.0, 'float', 
            'rotation around the Y axis for the right hip, in radian')
    add_data('dof_38', 0.0, 'float', 
            'rotation around the Z axis for the right hip, in radian')

    add_data('dof_39', 0.0, 'float',
            'rotation around the Z axis for the right knee, in radian')

    add_data('dof_40', 0.0, 'float',
            'rotation around the X axis for the right ankle, in radian')
    add_data('dof_41', 0.0, 'float',
            'rotation around the Y axis for the right ankle, in radian')
    add_data('dof_42', 0.0, 'float',
            'rotation around the Z axis for the right ankle, in radian')

    add_data('dof_43', 0.0, 'float',
             'rotation around the X axis for the left hip, in radian')
    add_data('dof_44', 0.0, 'float',
             'rotation around the Y axis for the left hip, in radian')
    add_data('dof_45', 0.0, 'float',
             'rotation around the Z axis for the left hip, in radian')

    add_data('dof_46', 0.0, 'float',
             'rotation around the Z axis for the left knee, in radian')

    add_data('dof_47', 0.0, 'float',
             'rotation around the X axis for the left ankle, in radian')
    add_data('dof_48', 0.0, 'float',
             'rotation around the Y axis for the left ankle, in radian')
    add_data('dof_49', 0.0, 'float',
             'rotation around the Z axis for the left ankle, in radian')

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            The second parameter should be the name of the object's parent. """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        logger.info('Component initialized')

    def _read_pose(self, armature):

        for channel in armature.channels:
            if 'X_' not in channel.name:
                logger.debug("\tChannel '%s': (%.4f, %.4f, %.4f)" %
                                        (channel,
                                         channel.joint_rotation[0],
                                         channel.joint_rotation[1],
                                         channel.joint_rotation[2]))
                if channel.name == 'Chest':
                    self.local_data['dof_12'] = channel.joint_rotation[2] #y
                    self.local_data['dof_13'] = channel.joint_rotation[0] #x
                    self.local_data['dof_14'] = channel.joint_rotation[1] #z
                if channel.name == 'Head':
                    self.local_data['dof_15'] = - channel.joint_rotation[0] #z axis
                    self.local_data['dof_16'] = channel.joint_rotation[2] #x axis
                    self.local_data['dof_17'] = channel.joint_rotation[1] #y axis

                if channel.name == 'UpArm.R':
                    self.local_data['dof_18'] = - channel.joint_rotation[1]
                    self.local_data['dof_19'] = - channel.joint_rotation[0]
                    self.local_data['dof_20'] = channel.joint_rotation[2]
                if channel.name == 'ForeArm.R':
                    self.local_data['dof_21'] = channel.joint_rotation[2]
                if channel.name == 'Hand.R':
                    self.local_data['dof_22'] = channel.joint_rotation[0]
                    self.local_data['dof_23'] = channel.joint_rotation[1]
                    self.local_data['dof_24'] = channel.joint_rotation[2]
                 
                if channel.name == 'UpArm.L':
                    self.local_data['dof_25'] = - channel.joint_rotation[1]
                    self.local_data['dof_26'] = - channel.joint_rotation[0]
                    self.local_data['dof_27'] = channel.joint_rotation[2]
                if channel.name == 'ForeArm.L':
                    self.local_data['dof_28'] = channel.joint_rotation[2]
                if channel.name == 'Hand.L':
                    self.local_data['dof_29'] = channel.joint_rotation[0]
                    self.local_data['dof_30'] = channel.joint_rotation[1]
                    self.local_data['dof_31'] = channel.joint_rotation[2]
                 
                if channel.name == 'UpLeg.R' :
                    self.local_data['dof_32'] = channel.joint_rotation[1]
                    self.local_data['dof_33'] = channel.joint_rotation[0]
                    self.local_data['dof_34'] = - channel.joint_rotation[2]
                if channel.name == 'LoLeg.R':
                    self.local_data['dof_35'] = channel.joint_rotation[0]
                if channel.name == 'Foot.R':
                    self.local_data['dof_36'] = channel.joint_rotation[0]
                    self.local_data['dof_37'] = channel.joint_rotation[1]
                    self.local_data['dof_38'] = channel.joint_rotation[2]
                 
                if channel.name == 'UpLeg.L':
                    self.local_data['dof_39'] = channel.joint_rotation[1]
                    self.local_data['dof_40'] = channel.joint_rotation[0]
                    self.local_data['dof_41'] = - channel.joint_rotation[2]
                if channel.name == 'LoLeg.L':
                    self.local_data['dof_42'] = channel.joint_rotation[0]
                if channel.name == 'Foot.L':
                    self.local_data['dof_43'] = channel.joint_rotation[0]
                    self.local_data['dof_44'] = channel.joint_rotation[1]
                    self.local_data['dof_45'] = channel.joint_rotation[2]


    def default_action(self):
        """ Extract the human posture """
        self.local_data['x'] = float(self.position_3d.x)
        self.local_data['y'] = float(self.position_3d.y)
        self.local_data['z'] = float(self.position_3d.z)
        self.local_data['yaw'] = float(self.position_3d.yaw) - (math.pi/2)
        self.local_data['pitch'] = float(self.position_3d.pitch)
        self.local_data['roll'] = float(self.position_3d.roll)

        self._read_pose(self.bge_object)

        logger.debug("LOCAL_DATA: ", self.local_data)
