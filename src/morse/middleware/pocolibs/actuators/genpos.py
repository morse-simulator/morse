import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_mw import init_extra_actuator
from morse.middleware.pocolibs.actuators.Genpos_Poster import ors_genpos_poster

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    init_extra_actuator(self, component_instance, function, mw_data, ors_genpos_poster)

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
