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
	if function_name == 'post_json_data':
		port_name = 'robots/{0}/{1}/out'.format(parent_name, component_name)
	elif function_name == 'read_json_data':
		port_name = 'robots/{0}/{1}/in'.format(parent_name, component_name)

	# Create the YARP port
	self.registerBufferedPortBottle([port_name])
	# Add the new method to the component
	component_instance.output_functions.append(function)
	# Store the name of the port
	self._component_ports[component_name] = port_name


def post_json_data(self, component_instance):
	""" Serialise the modified_data using JSON

	The argument is a copy of the component instance.
	This method will create a new dictionary with the contents
	of modified_data, and then encode it as JSON.
	"""
	port_name = self._component_ports[component_instance.blender_obj.name]

	new_data = {}
	i = 0;
	for var in component_instance.data_keys:
		new_data[var] = component_instance.modified_data[i]
		i = i + 1
	
	json_string = json.dumps(new_data)

	try:
		yarp_port = self.getPort(port_name)

		bottle = yarp_port.prepare()
		bottle.clear()

		# Go through the list of points
		# The list is the first item in ''modified_data''
		bottle.addString(json_string)

		yarp_port.write()
	except KeyError as detail:
		print ("ERROR: Specified port does not exist: ", detail)


def read_json_data(self, component_instance):
	""" Recover the data from a JSON string

	The argument is a copy of the component instance.
	This method will read a JSON string from the middleware,
	decode it and put the contents in the modified_data array.
	"""
	port_name = self._component_ports[component_instance.blender_obj.name]

	try:
		yarp_port = self.getPort(port_name)
		message_data = yarp_port.read(False)
	except KeyError as detail:
		print ("ERROR: Specified port does not exist: ", detail)

	new_data = json.loads(message_data)
	i = 0;
	for var in component_instance.data_keys:
		component_instance.modified_data[i] = new_data[var]
		i = i + 1
