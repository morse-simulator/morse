import sys, os
import GameLogic
import json

import helpers.MorseMath

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
	port_name = port_name + "/out"

	ob['Init_OK'] = False

	try:
		# Get the dictionary for the component's state
		robot_state_dict = GameLogic.robotDict[parent]
		#state_dict = GameLogic.componentDict[ob]
		ob['Init_OK'] = True
	except AttributeError:
		print ("Component Dictionary not found!")
		print ("This component must be part of a scene")

	if ob['Init_OK']:
		print ('######## GYROSCOPE INITIALIZATION ########')
		# Init the variables in the robot dictionary
		robot_state_dict['Yaw'] = 0.0
		robot_state_dict['Pitch'] = 0.0
		robot_state_dict['Roll'] = 0.0
		# Open the port
		GameLogic.orsConnector.registerBufferedPortBottle([port_name])
		print ('######## GYROSCOPE INITIALIZED ########')


def output(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	port_name = port_name + "/out"

	if ob['Init_OK']:
		robot_state_dict = GameLogic.robotDict[parent]

		############### Gyroscope ###################

		yaw, pitch, roll = helpers.MorseMath.euler_angle(ob)

		# Store the values in the robot's dictionary
		robot_state_dict['Yaw'] = yaw
		robot_state_dict['Pitch'] = pitch
		robot_state_dict['Roll'] = roll

		# Store the value in the sensor component's properties
		#  (for display using Blender Debug)
		ob['Yaw'] = yaw
		ob['Pitch'] = pitch
		ob['Roll'] = roll

		# Define the message structure to send.
		# It is a list of tuples (data, type).
		gyro_dict = {'yaw': yaw, 'pitch': pitch, 'roll': roll}
		message = json.dumps(gyro_dict)
		message_data = [ (message, 'string') ]

		#message_data = [ (gyro_angle, 'double') ]
		GameLogic.orsConnector.postMessage(message_data, port_name)


