import sys, os
import GameLogic
import json
# Import this library to recover the Python version
import platform

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

from middleware.independent.IndependentBlender import *

# Global variables for the names of the input and output ports
in_port_name = "admin/in"
out_port_name = "admin/out"


# Create a list of the robots in the scene
def Create_Dictionaries ():
	"""Creation of a list of all the robots and components in the scene.
	   Uses the properties of the objects to determine what they are."""

	# Create a dictioary of the components in the scene
	if not hasattr(GameLogic, 'componentDict'):
		GameLogic.componentDict = {}

	# Create a dictionary of the robots in the scene
	if not hasattr(GameLogic, 'robotDict'):
		GameLogic.robotDict = {}

	scene = GameLogic.getCurrentScene()

	# Get the robots
	for obj in scene.objects:
		try:
			obj['Robot_Tag']
			GameLogic.robotDict[obj] = {}
		except KeyError:
			pass
			#sys.exc_clear()  # Clears the last exception thrown

	# Get the components
	for obj, robot_state_dict in GameLogic.robotDict.items():
		# Create an empty list for the components of this robot
		robot_state_dict['components'] = []
		#print ("PARENT: {0}, GRAND_P {1}".format(obj, obj.parent))
		for child in obj.childrenRecursive:
			try:
				# Look for the components tagged as such
				#print ("\tchild: {0}".format(child))
				child['Component_Tag']
				#component_list.append (child['Component_Type'])
				#robot_state_dict['components'] = component_list
				robot_state_dict['components'].append (child['Component_Type'])

				# Create an empty dictionary for each component,
				#  and add it to GameLogic
				GameLogic.componentDict[child] = {}
			except KeyError:
				pass
				#sys.exc_clear()  # Clears the last exception thrown
		#print ("GameLogic[{0}] = {1}".format(name, obj))


def Check_Dictionaries():
	""" Print the contents of the robot and component dictionaries."""
	print ("------------------------------------")
	print ("GameLogic has the following robots:")
	for obj, robot_state_dict in GameLogic.robotDict.items():
		print ("\tROBOT: '{0}'".format(obj))
		for component in robot_state_dict['components']:
			print ("\t\t- Component: '{0}'".format(component))

	print ("GameLogic has the following components:")
	for obj, component_variables in GameLogic.componentDict.items():
		print ("\tCOMPONENT: '{0}'".format(obj))


# Create a port that publishes the list of components
# This will consist of 3 nested bottles:
##  1 Contains all the robots
##	2 Contains the name and the list of components of a robot
##  3 The list of components
def Publish_Bottled_Dictionaries(port_name):
	""" Prepare and send the dictionary data to a client program.
		This creates a set of nested bottles, to be sent by a yarp port."""
	if GameLogic.orsCommunicationEnabled:
		p = GameLogic.orsConnector.getPort(port_name)
		bottle = p.prepare()
		bottle.clear()

		# Create a structure of nested bottles
		for obj, robot_state_dict in GameLogic.robotDict.items():
			bottle2 = bottle.addList()
			bottle2.addString(obj.name)
			bottle3 = bottle2.addList()
			for component in robot_state_dict['components']:
				bottle3.addString(component.name)

		#...and send it
		p.write()



def Publish_JSON_Dictionaries(port_name):
	""" Prepare and send the dictionary data to a client program.
		Data is sent as a serialised JSON string."""
	if GameLogic.orsCommunicationEnabled:
		# Serialize the lists using JSON
		scene_elems = []
		for obj, robot_state_dict in GameLogic.robotDict.items():
			robot_list = [obj.name, robot_state_dict['components']]
			scene_elems.append (robot_list)
		message = json.dumps(scene_elems)

		print ("LIST: {0}".format(scene_elems))
		print ("JSON: {0}".format(message))
		print ("BACK: {0}".format(json.loads(message)))

		# Define the message structure to send.
		# It is a list of tuples (data, type).
		message_data = [ (message, 'string') ]
		GameLogic.orsConnector.postMessage(message_data, port_name)


def init(contr):
	""" Open the communication ports for administration."""
	print ('######## SCENE INITIALIZATION ########')
	print
	print ("Scripts path: " + scriptRoot)
	print ("Lib path: " + libRoot)
	print

	# Middleware initialization
	#if not hasattr(GameLogic, 'orsConnector'):
	GameLogic.orsConnector = MiddlewareConnector()

	GameLogic.orsCommunicationEnabled = True

	python_version = platform.python_version()
	print ("Python Version: {0}".format(python_version))
	GameLogic.pythonVersion = float(python_version[:3])


	print ('======== COMPONENT DICTIONARY INITIALIZATION =======')
	Create_Dictionaries()

	#print ("OPENING PORT '{0}'".format(in_port_name))
	#print ("OPENING PORT '{0}'".format(out_port_name))
	GameLogic.orsConnector.registerBufferedPortBottle([in_port_name, out_port_name])
	print ('======= COMPONENT DICTIONARY INITIALIZED =======')

	Check_Dictionaries()

	#Publish_JSON_Dictionaries(out_port_name)

def admin(contr):
	""" Respond to commands from the open communications port."""

	# Define a list with the data we are expecting
	data_types = ['string']
	data_list = GameLogic.orsConnector.readMessage(data_types, in_port_name)
	if data_list != None:
		command = data_list[0]

		print (" ===>> Communication with external agent established!")
		print ("Command:\t'{0}'".format(command))
		if command == "list_robots":
			print (" ===>> Sending list of elements")
			Publish_JSON_Dictionaries(out_port_name)


def finish(contr):
	"""Close the open ports."""
	sensor = contr.sensors['ESC_KEY']

	#execute only when the ESC key is released (if we don't test that,
	#the code get executed two time, when pressed, and when released)
	if not sensor.positive and sensor.triggered:
		print ('######### CLOSING PORTS... ########')

		GameLogic.orsConnector.finalize()

		quitActuator = contr.actuators['Quit_sim']
		contr.activate(quitActuator)

		print ('######### EXITING SIMULATION ########')



def restart(contr):
	"""Close the open ports."""
	sensor = contr.sensors['F11_KEY']

	#execute only when the F11 key is released (if we don't test that,
	#the code get executed two time, when pressed, and when released)
	if not sensor.positive and sensor.triggered:
		print ('######### CLOSING PORTS... ########')

		GameLogic.orsConnector.finalize()

		restartActuator = contr.actuators['Restart_sim']
		contr.activate(restartActuator)

		print ('######### RESTARTING SIMULATION ########')
