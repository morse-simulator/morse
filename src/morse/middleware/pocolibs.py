import sys
import array
import morse.helpers.middleware

class MorsePocolibsClass(morse.helpers.middleware.MorseMiddlewareClass):
	""" Handle communication between Blender and YARP."""

	def __init__(self, obj, parent=None):
		""" Initialize the network and connect to the yarp server."""
		self.blender_obj = obj
		# Will store the id's of posters, indexed by component name
		self._pocolilbs_posters = dict()
		self._component_ports = dict()


	def __del__(self):
		""" Destructor method.
			Close all open ports. """
		# Clear the posters open
		for component_name, poster_id in self._pocolilbs_posters.items():
			self.finalize(poster_id)


	def register_component(self, component_name,
			component_instance, mw_data):
		""" Open the port used to communicate the specified component.

		The name of the port is postfixed with in/out, according to
		the direction of the communication. """

		# Compose the name of the port
		parent_name = component_instance.robot_parent.blender_obj.name

		# Extract the information for this middleware
		# This will be tailored for each middleware according to its needs
		function_name = mw_data[1]
		poster_type = mw_data[2]
		poster_name = mw_data[3]


		# Choose what to do, depending on the poster type
		if poster_type == "genPos":
			from pocolibs.controllers.Control_Poster import ors_genpos_poster
			self._poster_dict[component_name] = ors_genpos_poster.locate_poster(poster_name)
			try:
				# Get the reference to the function
				function = getattr(self, function_name)
			except AttributeError as detail:
				print ("ERROR: %s. Check the 'component_config.py' file for typos" % detail)
				return
			component_instance.output_functions.append(function)

		elif poster_type == "pom":
			from pocolibs.sensors.Gyro_Poster import ors_pom_poster
			self._poster_dict[component_name] = ors_pom_poster.locate_poster(poster_name)
			try:
				# Get the reference to the function
				function = getattr(self, function_name)
			except AttributeError as detail:
				print ("ERROR: %s. Check the 'component_config.py' file for typos" % detail)
				return
			component_instance.output_functions.append(function)
	



	def read_genpos(self, component_instance):
		""" Read v,w from a genPos poster """
		genpos_speed = ors_genpos_poster.read_genPos_data(component_instance.blender_obj.name)
		#print ("Tuple type ({0}) returned".format(type(genpos_speed)))
		#print ("Tuple data: ({0}, {1})".format(genpos_speed.v, genpos_speed.w))

		component_instance.local_data['v'] = genpos_speed.v
		component_instance.local_data['w'] = genpos_speed.w


	def write_pom(self, component_instance):
		""" Write the position to a poaster

		The argument must be the instance to a morse gyroscope class. """

		# Compute the current time ( we only requiere that the pom date
		# increases using a constant step so real time is ok)
		t = datetime.now()
		pom_date = int(t.hour * 3600* 1000 + t.minute * 60 * 1000 +
				t.second * 1000 + t.microsecond / 1000)

		# Get the data from the gyroscope object
		robot = component_instance.robot_parent

		poster_id = self._poster_dict[component_instance.blender_obj.name]
		ors_pom_poster.post_data(poster_id, robot.x, robot.y, robot.z,
				robot.yaw, robot.pitch, robot.roll, pom_date)



	def read_message(self, component_instance):
		""" Read incoming data from a simple port.

		The argument is a copy of the component instance.
		Data is writen directly into the 'local_data' dictionary
		of the component instance.
		"""
		port_name = self._component_ports[component_instance.blender_obj.name]

		try:
			yarp_port = self.getPort(port_name)
			message_data = yarp_port.read(False)

			if message_data != None:
				# Data elements are of type defined in data_types
				i = 0
				for variable, data in component_instance.local_data.items():
					if isinstance(data, int):
						msg_data = message_data.get(i).asInt()
						component_instance.local_data[variable] = msg_data
					elif isinstance(data, float):
						msg_data = message_data.get(i).asDouble()
						component_instance.local_data[variable] = msg_data
					elif isinstance(data, basestring):
						msg_data = message_data.get(i).toString()
						component_instance.local_data[variable] = msg_data
					else:
						print ("Yarp ERROR: Unknown data type at 'read_message'")
					print ("READ VARIABLE {0} = {1}".format(variable, msg_data))
					i = i + 1

		except KeyError as detail:
			print ("ERROR: Specified port does not exist: ", detail)


	def post_message(self, component_instance):
		""" Send the data of a component through a simple port.

		The argument is a copy of the component instance.
		"""
		port_name = self._component_ports[component_instance.blender_obj.name]

		try:
			yarp_port = self.getPort(port_name)

			bottle = yarp_port.prepare()
			bottle.clear()
			# Data elements are tuples of (name, data)
			for variable, data in component_instance.send_data.items():
				if isinstance(data, int):
					bottle.addInt(data)
				elif isinstance(data, float):
					bottle.addDouble(data)
				elif isinstance(data, basestring):
					bottle.addString(data)
				else:
					print ("Yarp ERROR: Unknown data type at 'post_message'")

			#yarp_port.writeStrict()
			yarp_port.write()
		except KeyError as detail:
			print ("ERROR: Specified port does not exist: ", detail)


	def post_image_RGBA(self, component_instance):
		""" Send an RGBA image through the given named port."""
		port_name = self._component_ports[component_instance.blender_obj.name]

		try:
			yarp_port = self.getPort(port_name)
		except KeyError as detail:
			print ("ERROR: Specified port does not exist: ", detail)
			return

		if component_instance.blender_obj['capturing']:
			# Wrap the data in a YARP image
			img = self._yarp_module.ImageRgba()
			img.setTopIsLowIndex(0)
			img.setQuantum(1)

			# Get the image data from the camera instance
			img_string = component_instance.send_data['image']
			img_X = component_instance.image_size_X
			img_Y = component_instance.image_size_Y

			# Convert to an array object
			data = array.array('B',img_string)
			# Get the pointer to the data
			img_pointer = data.buffer_info()

			# Using Python pointer (converted or not)
			img.setExternal(img_pointer[0],img_X,img_Y)

			# Copy to image with "regular" YARP pixel order
			# Otherwise the image is upside-down
			img2 = yarp_port.prepare()
			img2.copy(img)

			# Write the image
			yarp_port.write()


	def finalize(self):
		""" Close all currently opened ports and release the network."""
		for port in self._yarpPorts.values():
			port.close()

		#self._network.fini()
		self._yarp_module.Network.fini()
		#yarp.Network.fini()
		print ('Yarp Mid: ports have been closed.')


	def getPort(self, portName):
		""" Retrieve a yarp port associated to the given name."""
		port = '/ors/' + portName
		return self._yarpPorts[port]


	def printOpenPorts(self):
		""" Return a list of all currently opened ports."""
		print ("Yarp Mid: List of ports:")
		for name, port in self._yarpPorts.items():
			print (" - Port name '{0}' = '{1}'".format(name, port))
