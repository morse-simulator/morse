import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
from morse.middleware.pocolibs.sensors.Target_Poster import ors_target_poster

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
        poster_name = 'target_{0}_{1}'.format(parent_name, component_name)

    poster_id = init_target_poster(self, component_instance, poster_name)
    if poster_id != None:
        logger.info("Pocolibs created poster '%s' of type target" % poster_id)
        component_instance.output_functions.append(function)
        # Store the name of the port
        self._poster_dict[component_name] = poster_id


def init_target_poster(self, component_instance, poster_name):
    """ Prepare the data for a target poster """
    # Measure the amount of coordinates that will be stored in a single array
    poster_id, ok = ors_target_poster.init_data(poster_name)
    if ok == 0:
        logger.error("Creating poster. The Target Pocolib export module may not work")
        return None

    logger.info("Target Poster '%s' created (ID: %d)" % (poster_name, poster_id))

    return poster_id


def write_target(self, component_instance):
    """ Write the target data to a Pocolibs poster.
    """
    target_dict = component_instance.local_data['victim_dict']
    poster_id = self._poster_dict[component_instance.blender_obj.name]

    if target_dict != {}:
        # if multiple target, just take the first one 
        for target in target_dict.values():
            return ors_target_poster.post_target_poster(poster_id, 1,  \
                        target['coordinate']['x'], target['coordinate']['y'])
    else:
        ors_target_poster.post_target_poster(poster_id, 0, 0.0, 0.0)


