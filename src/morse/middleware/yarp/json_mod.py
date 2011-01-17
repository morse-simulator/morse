import json

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name
    # Get the function name being inserted form the mw_data array
    function_name = mw_data[1]
    if function_name == 'post_json_message':
        port_name = 'robots/{0}/{1}/out'.format(parent_name, component_name)
    elif function_name == 'read_json_message':
        port_name = 'robots/{0}/{1}/in'.format(parent_name, component_name)

    # Create the YARP port
    self.registerBufferedPortBottle([port_name])
    # Add the new method to the component
    component_instance.output_functions.append(function)
    # Store the name of the port
    self._component_ports[component_name] = port_name


def post_json_message(self, component_instance):
    """ Serialise the local_data using JSON

    The argument is a copy of the component instance.
    This method will encode the 'local_data' dictionary as JSON.
    """
    port_name = self._component_ports[component_instance.blender_obj.name]

    json_string = json.dumps(component_instance.local_data)

    try:
        yarp_port = self.getPort(port_name)

        bottle = yarp_port.prepare()
        bottle.clear()
        bottle.addString(json_string)

        yarp_port.write()
    except KeyError as detail:
        print ("ERROR: Specified port does not exist: ", detail)


def read_json_message(self, component_instance):
    """ Recover the data from a JSON string

    The argument is a copy of the component instance.
    This method will read a JSON string from the middleware,
    decode it and put the contents in the local_data array.
    """
    port_name = self._component_ports[component_instance.blender_obj.name]

    try:
        yarp_port = self.getPort(port_name)
        message_data = yarp_port.read(False)
    except KeyError as detail:
        print ("ERROR: Specified port does not exist: ", detail)

    # Deserialise the data directly into the 'local_data' of the component
    component_instance.local_data = json.loads(message_data)
    """
    new_data = json.loads(message_data)
    i = 0;
    for var in component_instance.data_keys:
        component_instance.modified_data[i] = new_data[var]
        i = i + 1
    """
