import logging; logger = logging.getLogger("morse." + __name__)
import json
from collections import OrderedDict

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
        # Create the YARP port
        self.registerBufferedPortBottle([port_name])
        # Add the new method to the component
        component_instance.output_functions.append(function)
    elif function_name == 'read_json_message' or \
        function_name == 'read_json_waypoint':
        port_name = 'robots/{0}/{1}/in'.format(parent_name, component_name)
        # Create the YARP port
        self.registerBufferedPortBottle([port_name])
        # Add the new method to the component
        component_instance.input_functions.append(function)

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
    except KeyError as detail:
        logger.error("Specified port does not exist: ", detail)
        return

    bottle = yarp_port.prepare()
    bottle.clear()
    bottle.addString(json_string)

    yarp_port.write()


def read_json_message(self, component_instance):
    """ Recover the data from a JSON string

    The argument is a copy of the component instance.
    This method will read a JSON string from the middleware,
    decode it and put the contents in the local_data array.
    """
    port_name = self._component_ports[component_instance.blender_obj.name]

    try:
        yarp_port = self.getPort(port_name)
    except KeyError as detail:
        logger.error("Specified port does not exist: ", detail)
        return

    message_bottle = yarp_port.read(False)
    if message_bottle != None:
        message_data = message_bottle.get(0).toString()

        # Deserialise the data directly into a temporary ordered dictionary
        json_dict = json.loads(message_data, object_pairs_hook=OrderedDict)
        i = 0
        # Fill the component's 'local_data' dictionary,
        #  while also casting to the correct data types
        for variable, data in component_instance.local_data.items():
            if isinstance(data, int):
                component_instance.local_data[variable] = int(json_dict[variable])
            elif isinstance(data, float):
                component_instance.local_data[variable] = float(json_dict[variable])
            elif isinstance(data, str):
                component_instance.local_data[variable] = json_dict[variable]
            else:
                logger.error("Unknown data type at 'read_json_data'")
            i = i + 1


def read_json_waypoint(self, component_instance):
    """ Read a destination waypoint in the format used for ROSACE

    The structure read by JSON is expected to have two properties:
    point: structure with 'x', 'y', 'z'
    radius: the tolerance around those points
    """
    port_name = self._component_ports[component_instance.blender_obj.name]

    try:
        yarp_port = self.getPort(port_name)
    except KeyError as detail:
        logger.error("Specified port does not exist: ", detail)
        return

    message_bottle = yarp_port.read(False)
    if message_bottle != None:
        message_data = message_bottle.get(0).toString()

        # Deserialise the data directly into a temporary ordered dictionary
        json_dict = json.loads(message_data, object_pairs_hook=OrderedDict)

        point = json_dict['point']
        tolerance = json_dict['radius']
        component_instance.local_data['x'] = float(point['x'])
        component_instance.local_data['y'] = float(point['y'])
        component_instance.local_data['z'] = float(point['z'])
        component_instance.local_data['tolerance'] = float(tolerance)
        component_instance.local_data['speed'] = 3.0
