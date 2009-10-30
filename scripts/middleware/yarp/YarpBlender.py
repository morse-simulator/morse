import yarp

class YarpConnector:
	""" Handle communication between Blender and YARP."""

	def __init__(self):
		""" Initialize the network and connect to the yarp server."""
		
		self._yarpPorts = dict()

		yarp.Network.init()


	def registerBufferedPortBottle(self, portList):
		for portName in portList:
			portName = '/openrobots_simu/'+portName
			if portName not in self._yarpPorts:
				print '  * YARP: Adding ', portName, ' buffered bottle port.'
				port = yarp.BufferedPortBottle()
				port.open(portName)
				self._yarpPorts[portName] = port
			else: raise NameError(portName + " port name already exist!")

	def registerBufferedPortImageRgb(self, portList):
		for portName in portList:
			portName = '/openrobots_simu/'+portName
			if portName not in self._yarpPorts:
				print '  * YARP: Adding ', portName, ' buffered image port.'
				port = yarp.BufferedPortImageRgb()
				port.open(portName)
				self._yarpPorts[portName] = port
			else: raise NameError(portName + " port name already exist!")
		
	# Simple port. Necessary to stream video images from Blender
	def registerPort(self, portList):
		for portName in portList:
			portName = '/openrobots_simu/'+portName
			if portName not in self._yarpPorts:
				print '  * YARP: Adding ', portName, ' port.'
				port = yarp.Port()
				port.open(portName)
				self._yarpPorts[portName] = port
			else: raise NameError(portName + " port name already exist!")
	

	#Closes the ports and release the network
	def finalize(self):
		for port in self._yarpPorts.itervalues():
			port.close()
		
		yarp.Network.fini()
		print ' * YARP: ports have been closed.'
	
	def getPort(self, portName):
		port = '/openrobots_simu/' + portName
		return self._yarpPorts[port]


	def printOpenPorts(self):
		for name, port in self._yarpPorts.iteritems():
			print "Port name '{0}' = '{1}'".format(name, port)
