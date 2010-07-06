import socket

class SocketConnector:
	""" Handle communication between Blender and the outise world, using Sockets."""

	def __init__(self):
		""" Initialize the socket dictionary."""
		self._socketDict = dict()
		self._socketPorts = []
		self._host = ''
		self._base_port = 70000


	def openSocketsUDP(self, portList):
		""" Open one or more UDP sockets, given a list of names."""
		for portName in portList:
			portName = '/ors/' + portName
			if portName not in self._socketDict:
				print ("Socket Mid: Adding socket '{0}'.".format(portName))
				try:
					new_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
				except socket.error as msg:
					print ("Socket ERROR: Unable to create socket: '{0}'".format(msg))
					new_socket = None

				# Get the port number to use from the list
				# If none have been used, use a fixed number
				if self._socketPorts == []:
					socketPort = self._base_port
				# Otherwise, take the last port number, plus one
				else:
					socketPort = self._socketPorts[-1] + 1

				new_socket.bind((self._host, socketPort))
				#new_socket.setblocking(0)

				# Register the port in the dictionary
				self._socketDict[portName] = new_socket
				self._socketPorts.append(socketPort)

				print ("\t{0} : {1}".format(new_socket.getsockname(), socketPort))
				#print ("\t{0} : {1}".format(new_socket, socketPort))
			else:
				raise NameError(portName + " port name already exist!")


	def openSocketsTCP(self, portList):
		""" Open one or more TCP sockets, given a list of names."""
		for portName in portList:
			portName = '/ors/' + portName
			if portName not in self._socketDict:
				print ("Socket Mid: Adding socket '{0}'.".format(portName))
				try:
					new_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
				except socket.error as msg:
					print ("Socket ERROR: Unable to create socket: '{0}'".format(msg))
					new_socket = None

				# Get the port number to use from the list
				# If none have been used, use a fixed number
				if self._socketPorts == []:
					socketPort = self._base_port
				# Otherwise, take the last port number, plus one
				else:
					socketPort = self._socketPorts[-1] + 1

				new_socket.bind((self._host, socketPort))

				# Give the socket a larget timeout
				new_socket.settimeout(50)

				new_socket.setblocking(0)
				# Register the port in the dictionary
				self._socketDict[portName] = new_socket
				self._socketPorts.append(socketPort)

				print ("\t{0} : {1}".format(new_socket.getsockname(), socketPort))
			else:
				raise NameError(portName + " port name already exist!")


	def finalize(self):
		""" Closes the ports and release the network."""
		for ors_socket in self._socketDict.itervalues():
			ors_socket.close()
		print ('Socket Mid: sockets have been closed.')
	

	def getPort(self, portName):
		""" Retrieve the actual socket given its name."""
		port = '/ors/' + portName
		return self._socketDict[port]


	def readMessage(self, data_types, port_name, length=1024):
		""" Read a port expecting the data specified in data_types.
			The data is expected to be of a specified length."""
		try:
			in_socket = self.getPort(port_name)
			
			#(conn, addr) = in_socket.accept()
			#print ('Socket Mid: Connected by', addr)
			#data = in_socket.recv(length)
			#return data

			#data_string, addr = in_socket.recvfrom(4096)
			#print ("Socket Mid: Read data '{0}' from '{1}'", data_string, addr)
			#return data_string

		except KeyError as detail:
			print ("Socket ERROR: Specified port does not exist: ", detail)


	def postMessage(self, message_data, port_name):
		""" Send a message using a port."""
		try:
			out_socket = self.getPort(port_name)
			
			#(conn, addr) = out_socket.accept()
			#print ('Socket Mid: Connected by', addr)

			data_list = []

			# Data elements are tuples of (data, type)
			for element in message_data:
				msg_data, msg_type = element[0], element[1]

				##############################################################
				# Temporary solution to sending lists of elements via a socket
				##############################################################
				# Create a string containing all the data
				# The different elements will be separated by ';'
				data_list.append(msg_data)
				message = ";".join([`data` for data in data_list])
				#message = ", ".join(data_list) + "."

			#print ("Socket Mid: Going to send string: '{0}'".format(message))
			#out_socket.send(message)

		except KeyError as detail:
			print ("Socket ERROR: Specified port does not exist: ", detail)


	def postImage(self, img_pointer, img_X, img_Y, port_name):
		""" Send an RGB image using a port."""
		try:
			out_socket = self.getPort(port_name)
			
			(conn, addr) = out_socket.accept()
			print ('Socket Mid: Connected by', addr)
			out_socket.send(message)

		except KeyError as detail:
			print ("Socket ERROR: Specified port does not exist: ", detail)

	def printOpenPorts(self):
		""" Display a list of all currently opened sockets."""
		print ("Socket Mid: Currently opened sockets:")
		for name, socket in self._socketDict.iteritems():
			print (" - Port name '{0}' = '{1}'".format(name, socket))
