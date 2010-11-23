from morse.middleware.pocolibs.controllers.Control_Poster import ors_genpos_poster

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

    poster_id, ok = ors_genpos_poster.locate_poster(poster_name)
    # Use the value of 'ok' to determine if the poster was found
    if ok != 0:
        print ("Located 'genPos' poster. ID=%d" % poster_id)
        self._poster_in_dict[component_name] = poster_id
        component_instance.input_functions.append(function)
    else:
        print ("Poster 'genPos' not created. Component will not work")


def read_genpos(self, component_instance):
    """ Read v,w from a genPos poster """
    # Read from the poster specified
    poster_id = self._poster_in_dict[component_instance.blender_obj.name]
    genpos_speed = ors_genpos_poster.read_genPos_data(poster_id)
    #print ("Tuple type ({0}) returned".format(type(genpos_speed)))
    #print ("Tuple data: (%.4f, %.4f)" % (genpos_speed.v, genpos_speed.w))

    component_instance.modified_data[0] = genpos_speed.v
    component_instance.modified_data[1] = genpos_speed.w

    # Return true to indicate that a command has been received
    return True
