def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name
    port_name = 'robots/{0}/{1}/out'.format(parent_name, component_name)

    # Create the YARP port
    self.registerBufferedPortBottle([port_name])
    # Add the new method to the component
    component_instance.output_functions.append(function)
    # Store the name of the port
    self._component_ports[component_name] = port_name


def post_dictionary_data(self, component_instance):
    """ Serialise the data from a dictionary in YARP

    The argument is a copy of the component instance.
    This method will serialise the dictionary as a series of
    nested bottles.
    """
    port_name = self._component_ports[component_instance.blender_obj.name]

    try:
        yarp_port = self.getPort(port_name)

        bottle = yarp_port.prepare()
        bottle.clear()
        # Go through the list of points
        # The list is the first item in ''modified_data''
        for key, value in component_instance.modified_data[0].items():
            bottle2 = bottle.addList()
            bottle2.addString(key)
            bottle2.addDouble(value)

        yarp_port.write()
    except KeyError as detail:
        print ("ERROR: Specified port does not exist: ", detail)
