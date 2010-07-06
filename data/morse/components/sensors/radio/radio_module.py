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
	out_port_name = port_name + "/out"
	in_port_name = port_name + "/in"

	ob['Init_OK'] = False

	try:
		# Get the dictionary for the component's state
		robot_state_dict = GameLogic.robotDict[parent]
		ob['Init_OK'] = True
	except AttributeError:
		print ("Component Dictionary not found!")
		print ("This component must be part of a scene")

	if ob['Init_OK']:
		print ('######## RADIO INITIALIZATION ########')
		GameLogic.orsConnector.registerBufferedPortBottle([in_port_name, out_port_name])
		print ('######## RADIO INITIALIZED ########')


def output(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	out_port_name = port_name + "/out"
	in_port_name = port_name + "/in"

	if ob['Init_OK']:
		robot_state_dict = GameLogic.robotDict[parent]

		############### Thermometer ###################

		# Define a list with the data we are expecting
		data_types = ['string']
		data_list = GameLogic.orsConnector.readMessage(data_types, in_port_name)
		# Exit if there was no request for distances
		if data_list == None:
			return

		# Interpret the request received
		json_data = data_list[0]
		data = decode_message(json_data)

		try:
			new_range = data["range"]
			ob['Range'] = float(new_range)
			return
		except KeyError:
			pass
		
		try:
			new_range = data["request"]

			neighbour_robots = {}

			# Get the fire sources
			for robot in GameLogic.robotDict.keys():
				# Skip distance to self
				if parent != robot:
					distance = measure_distance_to_robot (parent, robot)
					if distance <= ob['Range']:
						neighbour_robots[robot.name] = distance

			send_robot_distance(neighbour_robots, out_port_name)
		except KeyError:
			pass


def measure_distance_to_robot(own_robot, target_robot):
	distance, globalVector, localVector = own_robot.getVectTo(target_robot)
	#print ("Distance from robot {0} to robot {1} = {2}".format(own_robot, target_robot, distance))
	return distance


def send_robot_distance(neighbour_robots, out_port_name):
	""" Send the dictionary with the neighbouring robots and distances to them."""
	# Define the message structure to send.
	# It is a list of tuples (data, type).
	neighbours = {'neighbours': neighbour_robots}
	message = json.dumps(neighbours)
	message_data = [ (message, 'string') ]
	GameLogic.orsConnector.postMessage(message_data, out_port_name)


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


