import socket
import cPickle
import morse.helpers.middleware

class MorseSocketClass(morse.helpers.middleware.MorseMiddlewareClass):
	""" External communication using sockets. """

	def __init__(self, obj, parent=None):
		""" Initialize the socket connections """
		self.blender_obj = obj
		self._socket_dict = dict()
		self._socket_ports = []
		self._host = ''
		self._base_port = 70000
		self._message_size = 1024

	def register_component(self, component_name, component_instance, mw_data):
		""" Open the port used to communicate the specified component.

		The name of the port is postfixed with in/out, according to
		the direction of the communication.
		"""
		# Compose the name of the port
		parent_name = component_instance.robot_parent.blender_obj.name
		port_name = 'robots/{0}/{1}'.format(parent_name, component_name)

		# Extract the information for this middleware
		# This will be tailored for each middleware according to its needs
		function_name = mw_data[1]

		try:
			# Get the reference to the function
			function = getattr(self, function_name)
		except AttributeError as detail:
			print ("ERROR: %s. Check the 'component_config.py' file for typos" % detail)
			return

		# Choose what to do, depending on the function being used
		# Data read functions
		if function_name == "read_message":
			port_name = port_name + '/in'
			self.open_UDP_server(component_name)
			component_instance.input_functions.append(function)
		"""
		# Data write functions
		elif function_name == "post_message":
			port_name = port_name + '/out'
			self.registerBufferedPortBottle([port_name])
			component_instance.output_functions.append(function)
		# Image write functions
		elif function_name == "post_image_RGBA":
			port_name = port_name + '/out'
			self.registerBufferedPortImageRgba([port_name])
			component_instance.output_functions.append(function)
		"""


	def open_UDP_server(self, component_name):
		""" Create an UDP server, given a list of names. """
		if component_name not in self._socket_dict:
			print ("Socket MW: Adding socket '{0}'.".format(component_name))
			try:
				new_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
			except socket.error as detail:
				print ("Socket ERROR: Unable to create socket: '{0}'".format(detail))
				return

			# Get the port number to use from the list
			# If none have been used, use a fixed number
			if self._socket_ports == []:
				socket_port = self._base_port
			# Otherwise, take the last port number, plus one
			else:
				socket_port = self._socket_ports[-1] + 1

			new_socket.bind((self._host, socket_port))
			new_socket.setblocking(0)

			# Register the port in the dictionary
			self._socket_dict[component_name] = new_socket
			self._socket_ports.append(socket_port)

			print ("Component {0} listening on port: {1}".format(component_name, socket_port))

		else:
			raise NameError(portName + " port name already exist!")




	def openSocketsUDP(self, portList):
		""" Open one or more UDP sockets, given a list of names."""
		for portName in portList:
			portName = '/ors/' + portName
			if portName not in self._socket_dict:
				print ("Socket Mid: Adding socket '{0}'.".format(portName))
				try:
					new_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
				except socket.error as msg:
					print ("Socket ERROR: Unable to create socket: '{0}'".format(msg))
					new_socket = None

				# Get the port number to use from the list
				# If none have been used, use a fixed number
				if self._socket_ports == []:
					socketPort = self._base_port
				# Otherwise, take the last port number, plus one
				else:
					socketPort = self._socket_ports[-1] + 1

				new_socket.bind((self._host, socketPort))
				#new_socket.setblocking(0)

				# Register the port in the dictionary
				self._socket_dict[portName] = new_socket
				self._socket_ports.append(socketPort)

				print ("\t{0} : {1}".format(new_socket.getsockname(), socket_port))
				#print ("\t{0} : {1}".format(new_socket, socketPort))
			else:
				raise NameError(portName + " port name already exist!")


	def openSocketsTCP(self, portList):
		""" Open one or more TCP sockets, given a list of names."""
		for portName in portList:
			portName = '/ors/' + portName
			if portName not in self._socket_dict:
				print ("Socket Mid: Adding socket '{0}'.".format(portName))
				try:
					new_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
				except socket.error as msg:
					print ("Socket ERROR: Unable to create socket: '{0}'".format(msg))
					new_socket = None

				# Get the port number to use from the list
				# If none have been used, use a fixed number
				if self._socket_ports == []:
					socketPort = self._base_port
				# Otherwise, take the last port number, plus one
				else:
					socketPort = self._socket_ports[-1] + 1

				new_socket.bind((self._host, socketPort))

				# Give the socket a larget timeout
				new_socket.settimeout(50)

				new_socket.setblocking(0)
				# Register the port in the dictionary
				self._socket_dict[portName] = new_socket
				self._socket_ports.append(socketPort)

				print ("\t{0} : {1}".format(new_socket.getsockname(), socketPort))
			else:
				raise NameError(portName + " port name already exist!")


	def finalize(self):
		""" Closes the ports and release the network."""
		for ors_socket in self._socket_dict.itervalues():
			ors_socket.close()
		print ('Socket Mid: sockets have been closed.')
	

	def read_message(self, component_instance):
		""" Read a port expecting the data specified in data_types.

		The data is expected to be of a specified length.
		Retuns True after reading data, or False if no data received. """
		# Get the reference to the correct socket
		in_socket = self._socket_dict[component_instance.blender_obj.name]

		try:
			message_data, CLIP = in_socket.recvfrom(self._message_size)
		except socket.error as detail:
			#print ("Socket ERROR: %s" % detail)
			return False

		pickled_data = cPickle.loads(message_data)

		# Extract the values from the socket data
		i = 0
		for data in component_instance.modified_data:
			msg_data = pickled_data[i]
			component_instance.modified_data[i] = msg_data

			print ("READ VARIABLE {0} = {1}".format(component_instance.data_keys[i], msg_data))
			i = i + 1

		return True


	def post_message(self, component_instance):
		""" Send a message using a port."""
		try:
			out_socket = self.getPort(port_name)
			
			#(conn, addr) = out_socket.accept()
			#print ('Socket Mid: Connected by', addr)

			data_list = []

			for msg_data in component_instance.modified_data:
				##############################################################
				# Temporary solution to sending lists of elements via a socket
				##############################################################
				# Create a string containing all the data
				# The different elements will be separated by ';'
				data_list.append(msg_data)
				message = ";".join([`data` for data in data_list])
				#message = ", ".join(data_list) + "."

			print ("Socket Mid: Going to send string: '{0}'".format(message))
			out_socket.send(message)

		except KeyError as detail:
			print ("Socket ERROR: Specified port does not exist: ", detail)


	def post_image(self, component_instance):
		""" Send an RGB image using a port."""
		try:
			out_socket = self.getPort(port_name)
			
			(conn, addr) = out_socket.accept()
			print ('Socket Mid: Connected by', addr)
			out_socket.send(message)

		except KeyError as detail:
			print ("Socket ERROR: Specified port does not exist: ", detail)


	def getPort(self, portName):
		""" Retrieve the actual socket given its name."""
		port = '/ors/' + portName
		return self._socket_dict[port]


	def printOpenPorts(self):
		""" Display a list of all currently opened sockets."""
		print ("Socket Mid: Currently opened sockets:")
		for name, socket in self._socket_dict.iteritems():
			print (" - Port name '{0}' = '{1}'".format(name, socket))
