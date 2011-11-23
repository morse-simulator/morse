import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
from morse.middleware.pocolibs.sensors.Platine_posture_Poster import ors_platine_posture_poster

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name
    # Check if the name of the poster has been given in mw_data
    try:
        # It should be the 4th parameter
        poster_name = mw_data[3]
    except IndexError as detail:
        # Compose the name of the poster, based on the parent and module names
        poster_name = 'platine_posture_{0}_{1}'.format(parent_name, component_name)

    poster_id = init_platine_posture_poster(self, component_instance, poster_name)
    if poster_id != None:
        logger.info("Pocolibs created poster '%s' of type platine_posture" % poster_id)
        component_instance.output_functions.append(function)
        # Store the name of the port
        self._poster_dict[component_name] = poster_id


def init_platine_posture_poster(self, component_instance, poster_name):
    """ Prepare the data for a platine_posture poster """
    # Measure the amount of coordinates that will be stored in a single array
    poster_id, ok = ors_platine_posture_poster.init_data(poster_name)
    if ok == 0:
        logger.error("Creating poster. The platine_posture Pocolib export module may not work")
        return None

    logger.info("platine_posture Poster '%s' created (ID: %d)" % (poster_name, poster_id))

    return poster_id


def write_platine_posture(self, component_instance):
    """ Write the platine_posture data to a Pocolibs poster.
    """
    poster_id = self._poster_dict[component_instance.blender_obj.name]

    ors_platine_posture_poster.post_platine_posture(poster_id, \
            component_instance.local_data['pan'], \
            component_instance.local_data['tilt'])

