from morse.middleware.pocolibs_mw import init_extra_actuator
from morse.middleware.pocolibs.actuators.Platine_Poster import ors_platine_poster

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    init_extra_actuator(self, component_instance, function, mw_data, ors_platine_poster)

def read_platine_(poster_id, component_instance):
    platine_data, ok = ors_platine_poster.read_platine_data(poster_id)

    if ok != 0:
        component_instance.local_data['pan'] = platine_data.yaw
        component_instance.local_data['tilt'] = platine_data.pitch

        # Return true to indicate that a command has been received
        return True
    else:
        return False

def read_platine(self, component_instance):
    """ Read pan,tilt from a platine poster """
    # Read from the poster specified
    poster_id = self._poster_in_dict[component_instance.blender_obj.name]
    read_platine_(poster_id, component_instance)

def read_platine_axis(self, component_instance):
    poster_id = self._poster_in_dict[component_instance.blender_obj.name]
    platine_data, ok = ors_platine_poster.read_platine_axis(poster_id)
    if ok != 0:
        component_instance.local_data['pan'] = platine_data.pan
        component_instance.local_data['tilt'] = platine_data.tilt

        # Return true to indicate that a command has been received
        return True
    else:
        return False

class PosterNotFound(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class PlatinePoster:
    def __init__(self, poster_name):
        self.poster_id = ors_platine_poster.createPosterHandler(poster_name)
        if not self.poster_id.found:
            raise PosterNotFound(poster_name)

    def read(self,  component_instance):
        read_platine_(self.poster_id, component_instance)

    def __del__(self):
        pass
