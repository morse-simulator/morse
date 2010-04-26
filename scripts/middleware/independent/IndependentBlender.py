import sys,os

try:
   scriptRoot = os.path.join(os.environ['ORS_ROOT'],'scripts')
except KeyError:
   scriptRoot = '.'

try:
   libRoot = os.path.join(os.environ['ORS_ROOT'],'lib')
except KeyError:
   libRoot = '.'

sys.path.append(scriptRoot)
sys.path.append(libRoot)

from middleware.yarp.YarpBlender import *
middleware = "yarp"

#from middleware.sockets.SocketsBlender import *
#middleware = "sockets"

class MiddlewareConnector:
	""" Handle communication between Blender and the chosen middleware."""

	def __init__(self):
		""" Initialize the network and connections."""
		if middleware is "yarp":
			self.connector = YarpConnector()
		elif middleware is "sockets":
			self.connector = SocketConnector()
		else:
			print ("Middleware ERROR: No middleware specified in 'IndependentBlender.py'")
			#raise NoMiddleware ("Middleware to use not defined")


	def registerBufferedPortBottle(self, portList):
		""" Open a buffered port bottle.
			Mainly used for yarp binding."""
		if middleware is "yarp":
			self.connector.registerBufferedPortBottle(portList)
		elif middleware is "sockets":
			self.connector.openSocketsUDP(portList)

	def registerBufferedPortImageMono(self, portList):
		""" Open a buffered port bottle, for image transmission.
			Currently not used."""
		if middleware is "yarp":
			self.connector.registerBufferedPortImageMono(portList)
		elif middleware is "sockets":
			self.connector.openSocketsTCP(portList)

	def registerBufferedPortImageRgb(self, portList):
		""" Open a buffered port bottle, for image transmission.
			Currently not used."""
		if middleware is "yarp":
			self.connector.registerBufferedPortImageRgb(portList)
		elif middleware is "sockets":
			self.connector.openSocketsTCP(portList)

	def registerBufferedPortImageRgba(self, portList):
		""" Open a buffered port bottle, for image transmission.
			Currently not used."""
		if middleware is "yarp":
			self.connector.registerBufferedPortImageRgba(portList)
		elif middleware is "sockets":
			self.connector.openSocketsTCP(portList)

	def registerPort(self, portList):
		""" Open a new communications port."""
		if middleware is "yarp":
			self.connector.registerPort(portList)
		elif middleware is "sockets":
			self.connector.openSocketsTCP(portList)

	def finalize(self):
		""" Closes the ports and release the network."""
		self.connector.finalize()
	
	def getPort(self, portName):
		""" Retrieve a communications port, given its name."""
		return self.connector.getPort(portName)

	def readMessage(self, data_types, port_name, length=1024):
		""" Read the specified data from a port."""
		return self.connector.readMessage(data_types, port_name, length)

	def postMessage(self, message_data, port_name):
		""" Send a message using a port."""
		self.connector.postMessage(message_data, port_name)

	def postImageRGB(self, img_pointer, img_X, img_Y, port_name):
		""" Send an RGB image using a port."""
		self.connector.postImageRGB(img_pointer, img_X, img_Y, port_name)

	def postImageRGBA(self, img_pointer, img_X, img_Y, port_name):
		""" Send an RGBA image using a port."""
		self.connector.postImageRGBA(img_pointer, img_X, img_Y, port_name)

	def printOpenPorts(self):
		""" Get the list of ports currently opened."""
		print ("Middleware: Currently opened ports:")
		self.connector.printOpenPorts()
