import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs.controllers.Genpos_Poster import ors_genpos_poster

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
        poster_name = '{0}_{1}'.format(parent_name, component_name)

    poster_id = ors_genpos_poster.createPosterHandler(poster_name)
    self._poster_in_dict[component_name] = poster_id
    component_instance.input_functions.append(function)

def read_genpos_(poster_id, component_instance):
    genpos_speed, ok = ors_genpos_poster.read_genPos_data(poster_id)
    logger.debug("Tuple type ({0}) returned".format(type(genpos_speed)))
    logger.debug("Tuple data: (%.4f, %.4f)" % (genpos_speed.v, genpos_speed.w))

    if ok != 0:
        component_instance.local_data['v'] = genpos_speed.v
        component_instance.local_data['w'] = genpos_speed.w
        return True
    else:
        return False

def read_genpos(self, component_instance):
    """ Read v,w from a genPos poster """
    # Read from the poster specified
    poster_id = self._poster_in_dict[component_instance.blender_obj.name]
    return read_genpos_(poster_id, component_instance)

class PosterNotFound(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class GenPosPoster:
    def __init__(self, poster_name):
        self.poster_id = ors_genpos_poster.createPosterHandler(poster_name)
        if not self.poster_id.found:
            raise PosterNotFound(poster_name)

    def read(self,  component_instance):
        return read_genpos_(self.poster_id, component_instance)

    def __del__(self):
        pass
