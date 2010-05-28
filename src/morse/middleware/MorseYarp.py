import yarp
import sys
import GameLogic

#class MorseYarpClass(MorseMiddleware.MorseMiddlewareClass):
class MorseYarpClass(object):
	""" Handle communication between Blender and YARP."""
	
	def __init__(self, obj, parent=None):
		""" Initialize the network and connect to the yarp server."""
		self.blender_obj = obj
		self._yarpPorts = dict()
		self._component_ports = dict()

		# Get the Network attribute of yarp,
		#  then call its init method
		# Strange that we should do all this,
		#  but it's the only way it seems to work
		self._yarp_module = sys.modules['yarp']
		#self._network = getattr(self._yarp_module, 'Network')
		#self._network.init()
		self._yarp_module.Network.init()

		#yarp.Network.init()

		#self.init_components()


	def __del__(self):
		""" Destructor method.
			Close all open ports. """
		self.finalize()


	def init_components(self):
		""" Binding to robotics components.

		Reads a dictionary of the components listed as using YARP.
		NOT USED RIGHT NOW.
		"""
		# Read the list of components and its associacted middleware
		# The file Component_Config.py is at the moment included
		#  in the .blend file of the scene
		import Component_Config
		# Add the hook functions to the appropriate components
		_component_list = Component_Config.component_mw
		for component_name, mw in _component_list.items():
			print ("Component: '%s' is operated by '%s'" % (component_name, mw))
			instance = GameLogic.componentDict['OB' + component_name]
			# Add the yarp function to the component's action list
			instance.action_functions.append(self.postMessage)

			port_name = 'robots/{0}/{1}'.format(instance.robot_parent.blender_obj.name, component_name)
			self.registerBufferedPortBottle([port_name])
			#instance.port_name = port_name
			self._component_ports[instance.blender_obj.name] = port_name


	def register_component(self, component_name,
			component_instance, io_direction):
		""" Open the port used to communicate the specified component.
		
		The name of the port is postfixed with in/out, according to
		the direction of the communication. """
		parent_name = component_instance.robot_parent.blender_obj.name
		port_name = 'robots/{0}/{1}/{2}'.format(parent_name,
				component_name, io_direction)
		self.registerBufferedPortBottle([port_name])
		#instance.port_name = port_name
		self._component_ports[component_name] = port_name


	def registerBufferedPortBottle(self, portList):
		""" Create a new Buffered Port Bottle, given an identifying name. """
		for portName in portList:
			portName = '/ors/'+portName
			if portName not in self._yarpPorts:
				#print ('Yarp Mid: Adding ' + portName + ' buffered bottle port.')
				port = self._yarp_module.BufferedPortBottle()
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
				port = self._yarp_module.BufferedPortImageMono()
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
				port = self._yarp_module.BufferedPortImageRgb()
				port.open(portName)


	def registerBufferedPortBottle(self, portList):
		""" Create a new Buffered Port Bottle, given an identifying name. """
		for portName in portList:
			portName = '/ors/'+portName
			if portName not in self._yarpPorts:
				#print ('Yarp Mid: Adding ' + portName + ' buffered bottle port.')
				port = self._yarp_module.BufferedPortBottle()
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
				port = self._yarp_module.BufferedPortImageMono()
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
				port = self._yarp_module.BufferedPortImageRgb()
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
				port = self._yarp_module.BufferedPortImageRgba()
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
				port = self._yarp_module.Port()
				port.open(portName)
				self._yarpPorts[portName] = port
			else: raise NameError(portName + " port name already exist!")
	

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
						print ("Yarp Mid ERROR: Unknown data type at 'read_message'")
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
			# Data elements are tuples of (name, data, type)
			for variable, data in component_instance.send_data.items():
				if isinstance(data, int):
					bottle.addInt(data)
				elif isinstance(data, float):
					bottle.addDouble(data)
				elif isinstance(data, basestring):
					bottle.addString(data)
				else:
					print ("Yarp Mid ERROR: Unknown data type at 'post_message'")

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
