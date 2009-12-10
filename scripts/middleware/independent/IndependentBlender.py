import sys,os

try:
   scriptRoot = os.path.join(os.environ['BLENDER_ROBOTICS_ROOT'],'scripts')
except KeyError:
   scriptRoot = '.'

try:
   libRoot = os.path.join(os.environ['BLENDER_ROBOTICS_ROOT'],'lib')
except KeyError:
   libRoot = '.'

sys.path.append(scriptRoot)
sys.path.append(libRoot)

import yarp
from middleware.yarp.YarpBlender import *
# from middleware.yarp import *

class MiddlewareConnector:
	""" Handle communication between Blender and YARP."""

	def __init__(self):
		""" Initialize the network and connect to the yarp server."""
		self.yarpConnector = YarpConnector()
		
		self.yarpConnector._yarpPorts = dict()

		yarp.Network.init()


	def registerBufferedPortBottle(self, portList):
		self.yarpConnector.registerBufferedPortBottle(portList)

		"""
		for portName in portList:
			portName = '/ors/'+portName
			if portName not in yarpConnector._yarpPorts:
				print ('  * Middleware: Adding ', portName, ' buffered bottle port.')
				port = yarp.BufferedPortBottle()
				port.open(portName)
				yarpConnector._yarpPorts[portName] = port
			else: raise NameError(portName + " port name already exist!")
		"""

	def registerBufferedPortImageRgb(self, portList):
		self.yarpConnector.registerBufferedPortImageRgb(portList)

		"""
		for portName in portList:
			portName = '/ors/'+portName
			if portName not in yarpConnector._yarpPorts:
				print ('  * Middleware: Adding ', portName, ' buffered image port.')
				port = yarp.BufferedPortImageRgb()
				port.open(portName)
				yarpConnector._yarpPorts[portName] = port
			else: raise NameError(portName + " port name already exist!")
		"""

	def registerPort(self, portList):
		self.yarpConnector.registerPort(portList)

	#Closes the ports and release the network
	def finalize(self):
		self.yarpConnector.finalize()
		"""
		for port in yarpConnector._yarpPorts.itervalues():
			port.close()
		
		yarp.Network.fini()
		print (' * Middleware: ports have been closed.')
		"""
	
	def getPort(self, portName):
		"""
		port = '/ors/' + portName
		return yarpConnector._yarpPorts[port]
		"""
		return self.yarpConnector.getPort(portName)


	# Send a message using a port
	def postMessage(self, message, port_name):
		try:
			yarp_port = self.yarpConnector.getPort(port_name)
			
			# prepare the YARP message...
			bottle = yarp_port.prepare()
			bottle.clear()
			bottle.addString(message)
			#...and send it
			yarp_port.writeStrict()
		except KeyError as detail:
			print ("ERROR: Specified port does not exist: ", detail)


	# Send an image using a port
	def postImage(self, img_pointer, img_X, img_Y, port_name):
		try:
			yarp_port = self.yarpConnector.getPort(port_name)

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

# nada = MiddlewareConnector()


	def printOpenPorts(self):
		print ("THIS SHOULD BE MY LIST OF PORTS")
		self.yarpConnector.printOpenPorts()
