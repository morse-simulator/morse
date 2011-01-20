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

    poster_id, ok = ors_platine_poster.locate_poster(poster_name)
    # Use the value of 'ok' to determine if the poster was found
    if ok != 0:
        print ("Located 'platine' poster. ID=%d" % poster_id)
        self._poster_in_dict[component_name] = poster_id
        component_instance.input_functions.append(function)
    else:
        print ("Poster 'platine' not found. Component will not work")


def read_platine(self, component_instance):
    """ Read pan,tilt from a platine poster """
    # Read from the poster specified
    poster_id = self._poster_in_dict[component_instance.blender_obj.name]
    platine_data = ors_platine_poster.read_platine_data(poster_id)
    #print ("Tuple type ({0}) returned".format(type(platine_data)))
    #print ("Tuple data: (%.4f, %.4f)" % (platine_data.pan, platine_data.tilt))

    component_instance.local_data['pan'] = platine_data.yaw
    component_instance.local_data['tilt'] = platine_data.pitch

    # Return true to indicate that a command has been received
    return True
