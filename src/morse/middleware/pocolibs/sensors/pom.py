import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs.sensors.Pom_Poster import ors_pom_poster

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
        poster_name = 'pom_{0}_{1}_Poster'.format(parent_name, component_name)

    poster_id = init_pom_poster(self, component_instance, poster_name)
    if poster_id != None:
        logger.info ("Pocolibs created poster of type pom")
        component_instance.output_functions.append(function)
        # Store the name of the port
        self._poster_dict[component_name] = poster_id


def init_pom_poster(self, component_instance, poster_name):
    """ Prepare the data for a pom poster """

    reference_frame = component_instance.blender_obj['reference_frame']
    confidence = component_instance.blender_obj['confidence']
    poster_id, ok = ors_pom_poster.init_data(poster_name, reference_frame, confidence)
    if ok == 0:
        logger.error("Ceating poster. This module may not work")
        return None

    logger.info("POM Poster '%s' created (ID: %d)" % (poster_name, poster_id))
    return poster_id


def write_pom(self, component_instance):
    """ Write the sensor position to a poster

    The argument must be the instance to a morse gyroscope class. """

    # Get the id of the poster already created
    poster_id = self._poster_dict[component_instance.blender_obj.name]
    ors_pom_poster.post_data(poster_id,
            component_instance.local_data.get('x', 0.0),
            component_instance.local_data.get('y', 0.0),
            component_instance.local_data.get('z', 0.0),
            component_instance.local_data.get('yaw', 0.0),
            component_instance.local_data.get('pitch', 0.0),
            component_instance.local_data.get('roll', 0.0))
