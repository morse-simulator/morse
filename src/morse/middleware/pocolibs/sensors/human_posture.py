import logging; logger = logging.getLogger("morse." + __name__)
import bge
from morse.middleware.pocolibs.sensors.Human_posture_Poster import ors_human_posture_poster

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
        poster_name = 'human_posture_{0}_{1}'.format(parent_name, component_name)

    poster_id = init_human_poster(self, component_instance, poster_name)
    if poster_id != None:
        logger.info("Pocolibs created poster '%s' of type human posture" % poster_id)
        component_instance.output_functions.append(function)
        # Store the name of the port
        self._poster_dict[component_name] = poster_id


def init_human_poster(self, component_instance, poster_name):
    """ Prepare the data for a human posture poster """

    poster_id, ok = ors_human_posture_poster.init_data(poster_name)
    if ok == 0:
        logger.error("Creating poster. The Human Posture Pocolib export module may not work")
        return None

    logger.info("Human Posture Poster '%s' created (ID: %d)" % (poster_name, poster_id))
    return poster_id


def export_posture(self, component_instance):
    """ Write the human posture to a Pocolibs poster.
    
    This method simply passes the values of the component_instance.local_data 
    dictionary that contains the 40 DOF of the human posture.

    The order of DOF relies on the order the keys where first inserted (cf 
    OrderedDict documentation).
    """

    # Get the id of the poster already created
    poster_id = self._poster_dict[component_instance.blender_obj.name]
    
    nb_dofs = 50
    
    #Special SWIG type declared in ors_human_posture_poster.i
    dofs = ors_human_posture_poster.doubleArray(nb_dofs) 
    
    raw_dofs = list(component_instance.local_data.values())
    
    logger.debug("Exporting posture: "+ str(raw_dofs))

    for i in range(nb_dofs):
        dofs[i] = raw_dofs[i]
    
    ors_human_posture_poster.post_human_poster(poster_id, dofs)
