import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
import morse.core.sensor
from morse.helpers.components import add_data

#logger.setLevel(logging.DEBUG)

class HumanPosture(morse.core.sensor.Sensor):
    """ 
    This sensor collects the positions of the bones in the human armature
    for the file ``$MORSE_ROOT/data/robots/mocap_human.blend``.

    It stores the position and orientation of the general armature
    object, as well as the local rotation of each individual bone. The
    rotation angles are given in radians. It exports the same interface than the
    :doc:`human posture sensor <../sensors/human_posture>`, but some
    joints are not reflected by the Kinect, and so they stay to their
    initial values.

    This sensor will only work for the ``mocap_human.blend`` model, as it
    uses a specific naming convention for each of the bones.

    You can also check to general documentation of the :doc:`human
    component <../others/human>`.

    .. image:: ../../../media/human_joints.png
      :align: center
      :width: 600
    """

    _name = "Human Posture (kinect version)"

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
             'elongation of the right arm, in meter (not computed)')

    add_data('dof_22', 0.0, 'float',
            'rotation around the Z axis for the right elbow, in radian')

    add_data('dof_23', 0.0, 'float', 'R_POINT (not computed)')

    add_data('dof_24', 0.0, 'float',
             'rotation around the X axis for the right wrist, in radian \
              (not computed)')
    add_data('dof_25', 0.0, 'float',
             'rotation around the Y axis for the right wrist, in radian \
              (not computed)')
    add_data('dof_26', 0.0, 'float',
             'rotation around the Z axis for the right write, in radian \
              (not computed')

    add_data('dof_27', 0.0, 'float',
            'rotation around the X axis for the left shoulder, in radian')
    add_data('dof_28', 0.0, 'float',
            'rotation around the Y axis for the left shoulder, in radian')
    add_data('dof_29', 0.0, 'float',
            'rotation around the Z axis for the left shoulder, in radian')

    add_data('dof_30', 0.0, 'float',
             'elongation of the left arm, in meter (not computed)')

    add_data('dof_31', 0.0, 'float',
             'rotation around the Z axis for the left elbow, in radian')

    add_data('dof_32', 0.0, 'float', 'L_POINT (not computed)')

    add_data('dof_33', 0.0, 'float', 
            'rotation around the X axis for the left wrist, in radian \
             (not computed)')
    add_data('dof_34', 0.0, 'float', 
            'rotation around the Y axis for the left wrist, in radian \
            (not computed)')
    add_data('dof_35', 0.0, 'float', 
            'rotation around the Z axis for the left wrist, in radian \
             (not computed)')

    add_data('dof_36', 0.0, 'float', 
            'rotation around the X axis for the right hip, in radian')
    add_data('dof_37', 0.0, 'float', 
            'rotation around the Y axis for the right hip, in radian')
    add_data('dof_38', 0.0, 'float', 
            'rotation around the Z axis for the right hip, in radian')

    add_data('dof_39', 0.0, 'float',
            'rotation around the Z axis for the right knee, in radian')

    add_data('dof_40', 0.0, 'float',
            'rotation around the X axis for the right ankle, in radian \
             (not computed)')
    add_data('dof_41', 0.0, 'float',
            'rotation around the Y axis for the right ankle, in radian \
             (not computed)')
    add_data('dof_42', 0.0, 'float',
            'rotation around the Z axis for the right ankle, in radian \
             (not compued)')

    add_data('dof_43', 0.0, 'float',
             'rotation around the X axis for the left hip, in radian')
    add_data('dof_44', 0.0, 'float',
             'rotation around the Y axis for the left hip, in radian')
    add_data('dof_45', 0.0, 'float',
             'rotation around the Z axis for the left hip, in radian')

    add_data('dof_46', 0.0, 'float',
             'rotation around the Z axis for the left knee, in radian')

    add_data('dof_47', 0.0, 'float',
             'rotation around the X axis for the left ankle, in radian \
              (not computed)')
    add_data('dof_48', 0.0, 'float',
             'rotation around the Y axis for the left ankle, in radian \
              (not computed)')
    add_data('dof_49', 0.0, 'float',
             'rotation around the Z axis for the left ankle, in radian \
              (not computed)')

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
                    self.local_data['dof_46'] = - channel.joint_rotation[2]
                # Kinect does not provide rotations for the feet
                #if channel.name == 'Foot.L':
                #    self.local_data['dof_47'] = channel.joint_rotation[0]
                #    self.local_data['dof_48'] = channel.joint_rotation[1]
                #    self.local_data['dof_49'] = channel.joint_rotation[2]



    def default_action(self):
        """ Extract the human posture """

        # Give the position of the Torso_Empty object as the position of
        # the human
        scene = blenderapi.scene()
        torso = scene.objects['Torso_Empty']
        self.local_data['x'] = torso.worldPosition[0]
        self.local_data['y'] = torso.worldPosition[1]
        self.local_data['z'] = torso.worldPosition[2]
        logger.debug("\tTorso_Empty position: (%.4f, %.4f, %.4f)" %
                     (torso.worldPosition[0], 
                      torso.worldPosition[1], 
                      torso.worldPosition[2]))

        # Pass also the rotation of the Torso_Empty
        self.local_data['yaw'] = torso.worldOrientation.to_euler().z
        self.local_data['pitch'] = torso.worldOrientation.to_euler().y
        self.local_data['roll'] = torso.worldOrientation.to_euler().x
        logger.debug("\tTorso_Empty orientation: (%.4f, %.4f, %.4f)" %
                       (self.local_data['roll'],
                        self.local_data['pitch'],
                        self.local_data['yaw']))

        self._read_pose(self.bge_object)

        logger.debug("LOCAL_DATA: %s" % self.local_data)
