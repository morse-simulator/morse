import sys, os
import GameLogic
import Mathutils
import math
import json

from Poster import poster

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
		posted = poster.init_data()
		print ('######## GYROSCOPE INITIALIZED ########')


def output(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	port_name = port_name + "/out"

	if ob['Init_OK']:
		robot_state_dict = GameLogic.robotDict[parent]

		############### Gyroscope ###################

		# Compute the angle with respect to the world
		rot_matrix = ob.worldOrientation
		#print_matrix (rot_matrix)

		# Angles determined directly from the rotation matrix.
		# Using the method described in:
		#  http://planning.cs.uiuc.edu/node103.html
		alpha = math.atan2 (rot_matrix[1][0], rot_matrix[0][0])
		beta = math.atan2 (-rot_matrix[2][0], math.sqrt(math.pow(rot_matrix[2][1], 2) + math.pow(rot_matrix[2][2], 2)))
		gamma = math.atan2 (rot_matrix[2][1], rot_matrix[2][2])

		yaw = math.degrees(alpha)
		roll = math.degrees(beta)
		pitch = math.degrees(gamma)

		#print ("Yaw: %.4f\tRoll: %.4f\tPitch: %.4f" % (yaw, roll, pitch))

		#### WARNING ####
		# This is probably a temporary solution, while Blender 2.5
		#  is made more stable. Currently it provides a rotation matrix
		#  with opposite signs to that of Blender 2.49b
		#
		# Change the signs of the angles
		#  if the Blender version is 2.5
		if GameLogic.pythonVersion >= 3:
			yaw = yaw * -1
			pitch = pitch * -1
			roll = roll * -1

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

		# Call to a SWIG method that will write a poster
		pos = ob.position
		posted = poster.post_data(pos[0], pos[1], pos[2], yaw, pitch, roll)
		#poster.fake()



def print_matrix (matrix):
	print ("OBJECT'S ROTATION MATRIX:")
	for row in matrix:
		line = "[%.4f %.4f %.4f]" % (row[0], row[1], row[2])
		print (line)
