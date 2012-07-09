import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_mw import init_extra_actuator
from morse.middleware.pocolibs.actuators.Niut_Poster import ors_niut_poster
import mathutils

# Assign constant variables to identify the joints of interest
# From niut/niutStruct.h
NIUT_HEAD = 1
NIUT_NECK = 2
NIUT_TORSO = 3
NIUT_WAIST = 4

NIUT_LEFT_COLLAR = 5
NIUT_LEFT_SHOULDER = 6
NIUT_LEFT_ELBOW = 7
NIUT_LEFT_WRIST = 8
NIUT_LEFT_HAND = 9
NIUT_LEFT_FINGERTIP = 10

NIUT_RIGHT_COLLAR = 11
NIUT_RIGHT_SHOULDER = 12
NIUT_RIGHT_ELBOW = 13
NIUT_RIGHT_WRIST = 14
NIUT_RIGHT_HAND = 15
NIUT_RIGHT_FINGERTIP = 16

NIUT_LEFT_HIP = 17
NIUT_LEFT_KNEE = 18
NIUT_LEFT_ANKLE = 19
NIUT_LEFT_FOOT = 20

NIUT_RIGHT_HIP = 21
NIUT_RIGHT_KNEE = 22
NIUT_RIGHT_ANKLE = 23
NIUT_RIGHT_FOOT = 24


# Define a transformation matrix for the position of the Kinect/Xtion sensor
transformation_matrix = mathutils.Matrix()
transformation_matrix.identity()


def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    init_extra_actuator(self, component_instance, function, mw_data, ors_niut_poster)
    logger.setLevel(logging.DEBUG)

    _create_transform_matrix()



def read_niut_ik_positions(self, component_instance):
    """ Read the positions of the joints in the niut poster """
    # Read from the poster specified
    poster_id = self._poster_in_dict[component_instance.blender_obj.name]

    result = True
    # Get the positions of the joints and multiply them by the matrix
    result = result and _store_joint_position(component_instance, poster_id, 'head_position', NIUT_HEAD, transformation_matrix)
    result = result and _store_joint_position(component_instance, poster_id, 'neck_position', NIUT_NECK, transformation_matrix)
    result = result and _store_joint_position(component_instance, poster_id, 'torso_position', NIUT_TORSO, transformation_matrix)
    result = result and _store_joint_position(component_instance, poster_id, 'left_hand_position', NIUT_LEFT_HAND, transformation_matrix)
    result = result and _store_joint_position(component_instance, poster_id, 'right_hand_position', NIUT_RIGHT_HAND, transformation_matrix)
    result = result and _store_joint_position(component_instance, poster_id, 'left_elbow_position', NIUT_LEFT_ELBOW, transformation_matrix)
    result = result and _store_joint_position(component_instance, poster_id, 'right_elbow_position', NIUT_RIGHT_ELBOW, transformation_matrix)
    result = result and _store_joint_position(component_instance, poster_id, 'left_shoulder_position', NIUT_LEFT_SHOULDER, transformation_matrix)
    result = result and _store_joint_position(component_instance, poster_id, 'right_shoulder_position', NIUT_RIGHT_SHOULDER, transformation_matrix)
    result = result and _store_joint_position(component_instance, poster_id, 'left_hip_position', NIUT_LEFT_HIP, transformation_matrix)
    result = result and _store_joint_position(component_instance, poster_id, 'right_hip_position', NIUT_RIGHT_HIP, transformation_matrix)
    result = result and _store_joint_position(component_instance, poster_id, 'left_knee_position', NIUT_LEFT_KNEE, transformation_matrix)
    result = result and _store_joint_position(component_instance, poster_id, 'right_knee_position', NIUT_RIGHT_KNEE, transformation_matrix)
    result = result and _store_joint_position(component_instance, poster_id, 'left_foot_position', NIUT_LEFT_FOOT, transformation_matrix)
    result = result and _store_joint_position(component_instance, poster_id, 'right_foot_position', NIUT_RIGHT_FOOT, transformation_matrix)

    # Return true to indicate that a command has been received
    return result


def _store_joint_position(component_instance, poster_id, ik_target, joint_index, transformation_matrix):
    """ Read the position of the pecified joint """
    joint_position, ok = ors_niut_poster.read_niut_joint_position(poster_id, joint_index)

    if ok != 0:
        # Convert the GEN_POINT_3D into a Blender vector
        position_vector = mathutils.Vector([joint_position.x, joint_position.y, joint_position.z])
        if transformation_matrix:
            new_position = position_vector * transformation_matrix
        else:
            new_position = position_vector

        component_instance.local_data[ik_target] = new_position

        #if ik_target == 'neck_position' or ik_target == 'torso_position':
        #    logger.debug("Joint '%s' (index=%d)" % (ik_target, joint_index))
        #    logger.debug("\toriginal : [%.4f, %.4f, %.4f] " % (position_vector[0], position_vector[1], position_vector[2]))
        #    logger.debug("\ttransform: [%.4f, %.4f, %.4f] " % ( \
        #        component_instance.local_data[ik_target][0],
        #        component_instance.local_data[ik_target][1],
        #        component_instance.local_data[ik_target][2]))

        return True
    else:
        return False


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
                [0.0, 0.0, 1.0, 0.0], \
                [1.0, 0.0, 0.0, 0.0], \
                [0.0, 1.0, 0.0, 0.0], \
                [0.0, 0.0, 0.0, 1.0]))

    # Additional rotation of the physical sensor, with respect to the Blender world
    # Currently set to 25.5 degrees around the Y axis
    kinect_rotation = mathutils.Matrix((
                [1.0,    0.0,    0.445,  0.0], \
                [0.0,    1.0,    0.0,    0.0], \
                [-0.445, 0.0,    1.0,    0.0], \
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
