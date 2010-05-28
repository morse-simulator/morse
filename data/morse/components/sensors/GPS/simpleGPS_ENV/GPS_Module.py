import sys, os
import GameLogic
import json

try:
   scriptRoot = os.path.join(os.environ['ORS_ROOT'],'scripts')
except KeyError:
   scriptRoot = '.'

try:
   libRoot = os.path.join(os.environ['ORS_ROOT'],'lib')
except KeyError:
   libRoot = '.'

if scriptRoot not in sys.path:
	sys.path.append(scriptRoot)
if scriptRoot not in sys.path:
	sys.path.append(libRoot)

from middleware.independent.IndependentBlender import *
import setup.ObjectData


def init(contr):
	# Middleware initialization
	if not hasattr(GameLogic, 'orsConnector'):
		GameLogic.orsConnector = MiddlewareConnector()

	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	gps_port_name = port_name + "/out"

	ob['Init_OK'] = False

	try:
		# Get the dictionary for the component's state
		state_dict = GameLogic.componentDict[ob]
		ob['Init_OK'] = True
	except AttributeError:
		print ("Component Dictionary not found!")
		print ("This component must be part of a scene")

	if ob['Init_OK']:
		print ('######## GPS INITIALIZATION ########')
		#print ("OPENING PORTS '{0}'".format(gps_port_name))
		GameLogic.orsConnector.registerBufferedPortBottle([gps_port_name])
		print ('######## GPS INITIALIZED ########')



def output(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	gps_port_name = port_name + "/out"

	if ob['Init_OK']:

		if GameLogic.orsCommunicationEnabled:
			x=ob.position[0];
			y=ob.position[1];
			z=ob.position[2];

			"""
			# Old style sending of the data as a bottle
			# Define the message structure to send.
			# It is a list of tuples (data, type).
			message_data = [ (x, 'double'), (y, 'double'), (z, 'double') ]
			GameLogic.orsConnector.postMessage(message_data, gps_port_name)
			"""

			# Sending the data as a JSON string
			position = {'x': x, 'y': y, 'z': z}
			message = json.dumps(position)
			message_data = [ (message, 'string') ]
			GameLogic.orsConnector.postMessage(message_data, gps_port_name)

			#print ("GPS ENV ", ob.name, " sent  x: ",x," y: ",y," z:", z)
			ob['Coordinates'] = "%.3f, %.3f, %.3f" % (x, y, z)
