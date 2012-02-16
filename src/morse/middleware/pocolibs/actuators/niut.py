import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_mw import init_extra_actuator
from morse.middleware.pocolibs.actuators.Niut_Poster import ors_niut_poster

# Assign constant variables to identify the joints of interest
# From niut/niutStruct.h
HEAD_INDEX = 1
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
    # Get head position
    result = result and _store_joint_position(component_instance, poster_id, 'head_position', HEAD_INDEX)
    # Get left hand position
    result = result and _store_joint_position(component_instance, poster_id, 'left_hand_position', LEFT_HAND_INDEX)
    # Get right hand position
    result = result and _store_joint_position(component_instance, poster_id, 'right_hand_position', RIGHT_HAND_INDEX)

    # Return true to indicate that a command has been received
    return result


def _store_joint_position(component_instance, poster_id, ik_target, joint_index):
    """ Read the position of the pecified joint """
    joint_position, ok = ors_niut_poster.read_niut_joint_position(poster_id, joint_index)

    if ok != 0:
        #component_instance.local_data[ik_target] = [joint_position.x, joint_position.y, joint_position.z]
        # Reorder the axis
        component_instance.local_data[ik_target] = [joint_position.z, joint_position.x, joint_position.y]
        #logger.debug("Joint '%s' (index=%d)\n\tposition: %s " % (ik_target, joint_index, component_instance.local_data[ik_target]))
        return True
    else:
        return False
