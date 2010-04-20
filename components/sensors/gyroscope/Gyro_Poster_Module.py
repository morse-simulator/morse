import sys, os
import GameLogic
import json
import datetime

from Gyro_Poster import ors_pom_poster
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

		# Start the external poster module
		poster_name = "morse_" + ob['Component_Type'] + "_poster"
		poster_name = poster_name.upper()
		robot_state_dict[port_name] = ors_pom_poster.init_data(poster_name)
		#robot_state_dict[port_name] = gyro_poster.init_data(poster_name)
		print ("Poster ID generated: {0}".format(robot_state_dict[port_name]))
		if robot_state_dict[port_name] == None:
			print ("ERROR creating poster. This module may not work")
			#ob['Init_OK'] = False

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

		# Compute the current time ( we only requiere that the pom date
		# increases using a constant step so real time is ok)
		t = datetime.datetime.now()
		date = int(t.hour * 3600* 1000 + t.minute * 60 * 1000 + 
				   t.second * 1000 + t.microsecond / 1000)

		# Call to a SWIG method that will write a poster
		pos = ob.position
		posted = ors_pom_poster.post_data(robot_state_dict[port_name], 
		#posted = gyro_poster.post_data(robot_state_dict[port_name], 
										  pos[0], pos[1], pos[2], 
										  yaw, pitch, roll, date)


def finish(contr):
	""" Procedures to kill the module when the program exits.
		12 / 04 / 2010
		Done for testing the closing of the poster. """

	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	port_name = port_name + "/out"
	robot_state_dict = GameLogic.robotDict[parent]

	print ("Closing poster with id: {0}".format(robot_state_dict[port_name]))
	#gyro_poster.finalize(robot_state_dict[port_name])
	ors_pom_poster.finalize(robot_state_dict[port_name])
	print ("Done!")

