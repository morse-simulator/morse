from morse.middleware.pocolibs.controllers.Platine_Poster import ors_platine_poster

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

    poster_id = ors_platine_poster.createPosterHandler(poster_name)
    self._poster_in_dict[component_name] = poster_id
    component_instance.input_functions.append(function)

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
