import yarp

class YarpConnector(MORSE_Middleware):
	""" Handle communication between Blender and YARP."""

	def __init__(self):
		""" Initialize the network and connect to the yarp server."""
		self._yarpPorts = dict()
		yarp.Network.init()

	def __del__(self):
		""" Destructor method.
			Close all open ports. """
		self.finalize()


	def registerBufferedPortBottle(self, portList):
		""" Create a new Buffered Port Bottle, given an identifying name. """
		for portName in portList:
			portName = '/ors/'+portName
			if portName not in self._yarpPorts:
				#print ('Yarp Mid: Adding ' + portName + ' buffered bottle port.')
				port = yarp.BufferedPortBottle()
				port.open(portName)
				self._yarpPorts[portName] = port
			else: raise NameError(portName + " port name already exist!")


	## XXX You need a special patch to export Mono Interface througth swig
	def registerBufferedPortImageMono(self, portList):
		""" Create a new Buffered Port Bottle, given an identifying name.
			This is exclusively used for image data."""
		for portName in portList:
			portName = '/ors/'+portName
			if portName not in self._yarpPorts:
				#print ('Yarp Mid: Adding ' + portName + ' buffered image port.')
				port = yarp.BufferedPortImageMono()
				port.open(portName)
				self._yarpPorts[portName] = port
			else: raise NameError(portName + " port name already exist!")

	def registerBufferedPortImageRgb(self, portList):
		""" Create a new Buffered Port Bottle, given an identifying name.
			This is exclusively used for image data."""
		for portName in portList:
			portName = '/ors/'+portName
			if portName not in self._yarpPorts:
				#print ('Yarp Mid: Adding ' + portName + ' buffered image port.')
				port = yarp.BufferedPortImageRgb()
				port.open(portName)
				self._yarpPorts[portName] = port
			else: raise NameError(portName + " port name already exist!")

	def registerBufferedPortImageRgba(self, portList):
		""" Create a new Buffered Port Bottle, given an identifying name.
			This is exclusively used for image data."""
		for portName in portList:
			portName = '/ors/'+portName
			if portName not in self._yarpPorts:
				#print ('Yarp Mid: Adding ' + portName + ' buffered image port.')
				port = yarp.BufferedPortImageRgba()
				port.open(portName)
				self._yarpPorts[portName] = port
			else: raise NameError(portName + " port name already exist!")
		
	def registerPort(self, portList):
		""" Open a simple yarp port.
			Used to send image data (Works better than a buffered port)."""
		for portName in portList:
			portName = '/ors/'+portName
			if portName not in self._yarpPorts:
				#print ('Yarp Mid: Adding ' + portName + ' port.')
				port = yarp.Port()
				port.open(portName)
				self._yarpPorts[portName] = port
			else: raise NameError(portName + " port name already exist!")
	

	def finalize(self):
		""" Close all currently opened ports and release the network."""
		for port in self._yarpPorts.values():
			port.close()
		
		yarp.Network.fini()
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


	def readMessage(self, data_types, port_name, length=1024):
		""" Read a port expecting the data specified in data_types.
			The data is expected to be of a specified length.
			The data read is returned in a list."""
		try:
			yarp_port = self.getPort(port_name)
			message_data = yarp_port.read(False)

			if message_data != None:
				# Data elements are of type defined in data_types
				i = 0
				data_list = []
				for msg_type in data_types:
					if msg_type == 'string':
						msg_data = message_data.get(i).toString()
					elif msg_type == 'int':
						msg_data = message_data.get(i).asInt()
					elif msg_type == 'double':
						msg_data = message_data.get(i).asDouble()
					else:
						print ("Yarp Mid ERROR: Unknown data type at 'readMessage'")
					data_list.append(msg_data)
					i = i + 1

				return data_list

		except KeyError as detail:
			print ("ERROR: Specified port does not exist: ", detail)



	def postMessage(self, message_data, port_name):
		""" Send a message through the given named port."""
		try:
			yarp_port = self.getPort(port_name)
			
			bottle = yarp_port.prepare()
			bottle.clear()
			# Data elements are tuples of (data, type)
			for element in message_data:
				msg_data, msg_type = element[0], element[1]
				if msg_type == 'string':
					bottle.addString(msg_data)
				elif msg_type == 'int':
					bottle.addInt(msg_data)
				elif msg_type == 'double':
					bottle.addDouble(msg_data)
				else:
					print ("Yarp Mid ERROR: Unknown data type at 'postMessage'")

			#yarp_port.writeStrict()
			yarp_port.write()
		except KeyError as detail:
			print ("ERROR: Specified port does not exist: ", detail)


	def postImageRGB(self, img_pointer, img_X, img_Y, port_name):
		""" Send an RGB image through the given named port."""
		try:
			yarp_port = self.getPort(port_name)

			# Wrap the data in a YARP image
			img = yarp.ImageRgb()
			img.setTopIsLowIndex(0)
			img.setQuantum(1)

			"""
			# Using Python pointer (converted or not)
			img.setExternal(img_pointer[0],img_X,img_Y)
			"""
			# Using the C pointer (converted)
			img.setExternal(img_pointer,img_X,img_Y)
			
			# Copy to image with "regular" YARP pixel order
			# Otherwise the image is upside-down
			img2 = yarp.ImageRgb()
			img2.copy(img)
			
			# Write the image
			yarp_port.write(img2)

		except KeyError as detail:
			print ("ERROR: Specified port does not exist: ", detail)


	def postImageRGBA(self, img_pointer, img_X, img_Y, port_name):
		""" Send an RGBA image through the given named port."""
		try:
			yarp_port = self.getPort(port_name)

			# Wrap the data in a YARP image
			img = yarp.ImageRgba()
			img.setTopIsLowIndex(0)
			img.setQuantum(1)

			# Using Python pointer (converted or not)
			img.setExternal(img_pointer[0],img_X,img_Y)
			"""
			# Using the C pointer (converted)
			img.setExternal(img_pointer,img_X,img_Y)
			"""
			
			# Copy to image with "regular" YARP pixel order
			# Otherwise the image is upside-down
			img2 = yarp_port.prepare()
			img2.copy(img)
			
			# Write the image
			yarp_port.write()

		except KeyError as detail:
			print ("ERROR: Specified port does not exist: ", detail)
