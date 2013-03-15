import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import PocolibsDataStreamInput
from niut.struct import *
from morse.core import mathutils

# Define a transformation matrix for the position of the Kinect/Xtion sensor
transformation_matrix = mathutils.Matrix()


class NiutPoster(PocolibsDataStreamInput):
    _type_name = "NIUT_HUMAN_LIST"

    _type_url = "http://trac.laas.fr/git/niut-genom/tree/niutStruct.h#n82"
    def initialize(self):
        PocolibsDataStreamInput.initialize(self, NIUT_HUMAN_LIST)

        self.couples = \
                  [('head_position', NIUT_HEAD),
                   ('neck_position', NIUT_NECK),
                   ('torso_position', NIUT_TORSO),
                   ('left_hand_position', NIUT_LEFT_HAND),
                   ('right_hand_position', NIUT_RIGHT_HAND),
                   ('left_elbow_position', NIUT_LEFT_ELBOW),
                   ('right_elbow_position', NIUT_RIGHT_ELBOW),
                   ('left_shoulder_position', NIUT_LEFT_SHOULDER),
                   ('right_shoulder_position', NIUT_RIGHT_SHOULDER),
                   ('left_hip_position', NIUT_LEFT_HIP),
                   ('right_hip_position', NIUT_RIGHT_HIP),
                   ('left_knee_position', NIUT_LEFT_KNEE),
                   ('right_knee_position', NIUT_RIGHT_KNEE),
                   ('left_foot_position', NIUT_LEFT_FOOT),
                   ('right_foot_position', NIUT_RIGHT_FOOT)]

        _create_transform_matrix()

    def _store_joint_position(self, joints, ik_target, joint_index):
        joint_position = joints[joint_index].position
        # Convert the GEN_POINT_3D into a Blender vector
        position_vector = mathutils.Vector([joint_position.x, joint_position.y, joint_position.z])
        if transformation_matrix:
            #logger.error("Kinect position: [%.4f, %.4f, %.4f]" % (transformation_matrix[0][3], transformation_matrix[1][3], transformation_matrix[2][3]))
            new_position = position_vector * transformation_matrix
            new_position = new_position + mathutils.Vector([transformation_matrix[0][3], transformation_matrix[1][3], transformation_matrix[2][3]])
        else:
            new_position = position_vector

        self.data[ik_target] = new_position


    def default(self, ci = ''):
        human_list = self.read()
        if human_list:
            for i in range(0, 16):
                if human_list.users[i].state == NIUT_TRACKING:
                    joints = human_list.users[i].skeleton.joint
                    for target, idx in self.couples:
                        self._store_joint_position(joints, target, idx)
                    return

def _create_transform_matrix():
    """ Construct the transformation matrix
    from the Kinect to the Blender frame of reference
    """
    global transformation_matrix

	#         Y  X                  Z  Y
	#         | /                   | /
	# KinCam  |/ ____ Z  ,   World  |/_____X
    # Transformation of the Kinect frame of reference to that of Blender
    kinect_matrix = mathutils.Matrix((
                [0.0, 1.0, 0.0, 0.0], \
                [0.0, 0.0, 1.0, 0.0], \
                [1.0, 0.0, 0.0, 0.0], \
                [0.0, 0.0, 0.0, 1.0]))

    # Additional rotation of the physical sensor, with respect to the Blender world
    # Currently set to 25.5 degrees around the Y axis
    kinect_rotation = mathutils.Matrix((
                [1.0,    0.0,    -0.445, 0.0], \
                [0.0,    1.0,    0.0,    0.0], \
                [0.445,  0.0,    1.0,    0.0], \
                [0.0,    0.0,    0.0,    1.0]))

    # Spin the positions around the Z axis, to match with the Blender human
    rotation_matrix = mathutils.Matrix((
                [-1.0, 0.0, 0.0, 0.0], \
                [0.0, -1.0, 0.0, 0.0], \
                [0.0, 0.0, 1.0, 0.0], \
                [0.0, 0.0, 0.0, 1.0]))

    # Position of the kinect with respect to the human.
    # XXX: Make this adjustable from the real position in the scene
    kinect_position = [2.0, 0.0, 2.0]
    #logger.error("Kinect position: [%.4f, %.4f, %.4f]" % (kinect_position[0], kinect_position[1], kinect_position[2]))

    transformation_matrix = kinect_matrix * kinect_rotation * rotation_matrix
    # Add the position of the Kinect sensor
    transformation_matrix[0][3] = kinect_position[0]
    transformation_matrix[1][3] = kinect_position[1]
    transformation_matrix[2][3] = kinect_position[2]
