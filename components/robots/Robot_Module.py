import sys, os
import GameLogic
import json
import re


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

	motion_act = contr.actuators['motion_actuator']

	#sets the number of frames used to reach the target velocity
	motion_act.damping = 10

	#sets to move & rotate the object using the game object (local) axis.
	motion_act.useLocalDLoc = 1
	motion_act.useLocalDRot = 1

	# Get the current object
	ob, port_name = setup.ObjectData.get_robot_data(contr)

	# Get the dictionary for the robot's state
	robot_state_dict = GameLogic.robotDict[ob]

	# Add the speed values to the dictionary
	robot_state_dict['vx'] = 0.0
	robot_state_dict['vy'] = 0.0
	robot_state_dict['vz'] = 0.0

	# Add the rotation values to the dictionary
	robot_state_dict['rx'] = 0.0
	robot_state_dict['ry'] = 0.0
	robot_state_dict['rz'] = 0.0

	# Add movement status to the dictionary
	robot_state_dict['moveStatus'] = "Stop"

	print
	print ('######## ROBOT INITIALIZATION ########')
	#print ("OPENING PORTS '{0}'".format(port_name))
	in_port_name = port_name + "/in"
	out_port_name = port_name + "/out"
	GameLogic.orsConnector.registerBufferedPortBottle([in_port_name, out_port_name])
	#GameLogic.orsConnector.printOpenPorts()
	print ('######## ROBOT INITIALIZED ########')


def move(contr):
	""" Change the robot's Motion actuator
		according to the data in the robot's dictionary."""
	ob = contr.owner

	sensor = contr.sensors['Speed_msg']
	if sensor.positive:

		""" NOT WORKING. (THE 'return' MAKES THE ROBOT MOVEMENT NOT UPDATED)
		# Exit if the message is not addressed to this object
		for msg in sensor.bodies:
			if msg != ob.name:
				return
		"""

		# Get the dictionary for the robot's state
		robot_state_dict = GameLogic.robotDict[ob]

		# Get the speed values from the dictionary
		vx = robot_state_dict['vx']
		vy = robot_state_dict['vy']
		vz = robot_state_dict['vz']
		#print ("{0} SETTING SPEED: {1}, {2}, {3}".format(ob.name, vx, vy, vz))

		# Get the rotation values from the dictionary
		rx = robot_state_dict['rx']
		ry = robot_state_dict['ry']
		rz = robot_state_dict['rz']
		#print ("{0} SETTING ROTATION: {1}, {2}, {3}".format(ob.name, rx, vy, rz))

		motion_act = contr.actuators['motion_actuator']
		motion_act.dLoc = (vx, vy, vz) #Blender  > 2.49
		motion_act.dRot = (rx, ry, rz) #Blender  > 2.49
		contr.activate(motion_act)


def report(contr):
	""" Respond to requests for the robot status."""

	# Get the current object
	ob, port_name = setup.ObjectData.get_robot_data(contr)
	in_port_name = port_name + "/in"
	out_port_name = port_name + "/out"

	# Get the dictionary for the robot's state
	robot_state_dict = GameLogic.robotDict[ob]

	# Check if there has been a change of status event
	sensor = contr.sensors['Status_msg']
	if sensor.positive:
		send_robot_status(robot_state_dict, out_port_name)

	# Define a list with the data we are expecting
	data_types = ['string']
	data_list = GameLogic.orsConnector.readMessage(data_types, in_port_name)
	if data_list != None:
		#command = data_list[0]

		json_data = data_list[0]
		data = decode_message(json_data)
		command = data["command"]

		print (" ==>> Communication with external agent established!")
		print ("Command:\t'{0}'".format(command))
		if command == "status":
			print (" ===>> Sending Robot status")
			send_robot_status(robot_state_dict, out_port_name)

		elif command == "move":
			print (" ===>> Starting movement")
			robot_state_dict['moveStatus'] = "Moving to 0"
			send_robot_status(robot_state_dict, out_port_name)

		elif command == "stop":
			print (" ===>> Stopping robot")
			robot_state_dict['moveStatus'] = "Stop"
			send_robot_status(robot_state_dict, out_port_name)


def decode_message(json_data):
	""" Decode a data structure using JSON.
		The data is initially a string.
		Returns a Python object,
		either an int, double, string, list or dictionary."""
	# Remove the quotations at the start and end of the string
	#json_data = re.sub(r'"(.*)"', r'\1', json_data)
	# Unescape all other quotation marks
	#json_data = re.sub(r'\\(.)', r'\1', json_data)
	clean_data = json.loads(json_data, encoding='UTF-8')

	return clean_data


def send_robot_status(robot_state_dict, out_port_name):
	# Define the message structure to send.
	# It is a list of tuples (data, type).
	state = {'state': robot_state_dict['moveStatus']}
	message = json.dumps(state)
	message_data = [ (message, 'string') ]

	#message_data = [ (robot_state_dict['moveStatus'], 'string') ]
	GameLogic.orsConnector.postMessage(message_data, out_port_name)
