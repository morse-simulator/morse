import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_mw import init_extra_actuator
from morse.middleware.pocolibs.actuators.Niut_Poster import ors_niut_poster

# Assign constant variables to identify the joints of interest
# From niut/niutStruct.h
HEAD_INDEX = 1
TORSO_INDEX = 3
LEFT_HAND_INDEX = 9
RIGHT_HAND_INDEX = 15

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    init_extra_actuator(self, component_instance, function, mw_data, ors_niut_poster)
    logger.setLevel(logging.DEBUG)

def read_niut_ik_positions(self, component_instance):
    """ Read the positions of the head and hands in the niut poster """
    # Read from the poster specified
    poster_id = self._poster_in_dict[component_instance.blender_obj.name]

    result = True
    # Get torso position. This will be used as the reference point
    #  for the rest of the targets
    result = result and _store_joint_position(component_instance, poster_id, 'torso_position', TORSO_INDEX, None)
    torso_position = component_instance.local_data['torso_position']
    #logger.error("torso (index=%d)\n\tposition: [%.4f, %.4f, %.4f]" % (TORSO_INDEX, torso_position[0], torso_position[1], torso_position[2]))

    # Get head position
    result = result and _store_joint_position(component_instance, poster_id, 'head_position', HEAD_INDEX, torso_position)
    # Get left hand position
    result = result and _store_joint_position(component_instance, poster_id, 'left_hand_position', LEFT_HAND_INDEX, torso_position)
    # Get right hand position
    result = result and _store_joint_position(component_instance, poster_id, 'right_hand_position', RIGHT_HAND_INDEX, torso_position)

    # Indicate the rotation expected of the torso
    component_instance.local_data['torso_orientation'] = [-1.5708, 0.0, -1.5708]
    #component_instance.local_data['torso_orientation'] = [3.14159, -1.5708, 0.0]

    # Return true to indicate that a command has been received
    return result


def _store_joint_position(component_instance, poster_id, ik_target, joint_index, reference_position):
    """ Read the position of the pecified joint """
    joint_position, ok = ors_niut_poster.read_niut_joint_position(poster_id, joint_index)

    if ok != 0:
        """
        if reference_position:
            logger.debug("Joint '%s' (index=%d)\n\tposition: [%.4f, %.4f, %.4f]" % (ik_target, joint_index, joint_position.x, joint_position.y, joint_position.z))
            # Get the positions of the joints with respect to the torso
            joint_position.x = joint_position.x - reference_position[0]
            joint_position.y = joint_position.y - reference_position[1]
            joint_position.z = joint_position.z - reference_position[2]
            logger.debug("Joint '%s' (index=%d) CORRECTED\n\tposition: [%.4f, %.4f, %.4f]" % (ik_target, joint_index, joint_position.x, joint_position.y, joint_position.z))
        """

        component_instance.local_data[ik_target] = [joint_position.x, joint_position.y, joint_position.z]
        # Reorder the axis
        #component_instance.local_data[ik_target] = [joint_position.y, joint_position.x, joint_position.z]
        #logger.debug("Joint '%s' (index=%d)\n\tposition: %s " % (ik_target, joint_index, component_instance.local_data[ik_target]))
        return True
    else:
        return False
