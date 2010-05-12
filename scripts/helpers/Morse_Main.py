import sys
import GameLogic
import re
import json
# Import this library to recover the Python version
import platform


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

	# Create a dictionary with the middlewares used
	if not hasattr(GameLogic, 'mwDict'):
		GameLogic.mwDict = {}

	scene = GameLogic.getCurrentScene()

	# Get the robots
	for obj in scene.objects:
		try:
			obj['Robot_Tag']
			# Create an object instance and store it
			instance = Create_Instance (obj)
			GameLogic.robotDict[obj] = instance
		except KeyError:
			pass
			#sys.exc_clear()	# Clears the last exception thrown
								# Does not work in Python 3

	# Get the robot and its instance
	for obj, robot_instance in GameLogic.robotDict.items():
		# Create an empty list for the components of this robot
		robot_instance.components = []
		for child in obj.childrenRecursive:
			try:
				# Look for the components tagged as such
				child['Component_Tag']
				robot_instance.components.append (child)

				# Create an instance of the component class
				#  and add it to the component list of GameLogic
				instance = Create_Instance (child, robot_instance)
				GameLogic.componentDict[child] = instance

			except KeyError:
				pass
				#sys.exc_clear()	# Clears the last exception thrown
									# Does not work in Python 3


	# Get the middlewares
	for obj in scene.objects:
		try:
			obj['Middleware_Tag']
			# Create an object instance and store it
			instance = Create_Instance (obj)
			GameLogic.mwDict[obj] = instance
		except KeyError:
			pass
			#sys.exc_clear()	# Clears the last exception thrown
								# Does not work in Python 3



def Check_Dictionaries():
	""" Print the contents of the robot and component dictionaries."""
	print ("------------------------------------")
	print ("GameLogic has the following robots:")
	for obj, robot_instance in GameLogic.robotDict.items():
		print ("\tROBOT: '{0}'".format(obj))
		for component in robot_instance.components:
			print ("\t\t- Component: '{0}'".format(component))

	print ("\nGameLogic has the following components:")
	for obj, component_variables in GameLogic.componentDict.items():
		print ("\tCOMPONENT: '{0}'".format(obj))

	print ("\nGameLogic has the following middlewares:")
	for obj, mw_variables in GameLogic.mwDict.items():
		print ("\tMIDDLEWARE: '{0}'".format(obj))


def Create_Instance(obj, parent=None):
	""" Dynamically load a Python module and create an instance object
		of the class defined within. """
	# Read the path and class of the object from the Logic Properties
	source_file = obj['Path'] + obj['Class']
	module_name = re.sub('/', '.', source_file)
	print ("Path to Component Class: %s" % module_name)
	# Import the module containing the class
	__import__(module_name)
	module = sys.modules[module_name]
	# Create an instance of the object class
	klass = getattr(module, obj['Class'] + '_Class')
	instance = klass(obj, parent)

	return instance



def Publish_JSON_Dictionaries(port_name):
	""" Prepare and send the dictionary data to a client program.
		Data is sent as a serialised JSON string."""
	# Serialize the lists using JSON
	scene_elems = []
	for obj, robot_instance in GameLogic.robotDict.items():
		robot_list = [obj.name, robot_instance.components]
		scene_elems.append (robot_list)
	message = json.dumps(scene_elems)

	print ("LIST: {0}".format(scene_elems))
	print ("JSON: {0}".format(message))
	print ("BACK: {0}".format(json.loads(message)))

	# Define the message structure to send.
	# It is a list of tuples (data, type).
	message_data = [ (message, 'string') ]
	#GameLogic.orsConnector.postMessage(message_data, port_name)


def init(contr):
	""" Open the communication ports for administration."""
	print ('\n######## SCENE INITIALIZATION ########')
	# Get the version of Python used, according to the pythonpath
	# This is used to determine also the version of Blender
	python_version = platform.python_version()
	print ("Python Version: {0}".format(python_version))
	# Chop the version to only 3 chars: #.#  and convert to a number
	GameLogic.pythonVersion = float(python_version[:3])

	print ('======== COMPONENT DICTIONARY INITIALIZATION =======')
	Create_Dictionaries()

	#print ("OPENING PORT '{0}'".format(in_port_name))
	#print ("OPENING PORT '{0}'".format(out_port_name))
	#GameLogic.orsConnector.registerBufferedPortBottle([in_port_name, out_port_name])
	print ('======= COMPONENT DICTIONARY INITIALIZED =======')

	Check_Dictionaries()

	#Publish_JSON_Dictionaries(out_port_name)


def admin(contr):
	""" Respond to commands from the open communications port."""

	"""
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
	"""


def finish(contr):
	"""Close the open ports."""
	sensor = contr.sensors['ESC_KEY']

	#execute only when the ESC key is released (if we don't test that,
	#the code get executed two time, when pressed, and when released)
	if not sensor.positive and sensor.triggered:
		print ('######### CLOSING PORTS... ########')

		#GameLogic.orsConnector.finalize()

		# Force the deletion of the sensor objects
		for obj, component_instance in GameLogic.componentDict.items():
			del obj

		# Force the deletion of the robot objects
		for obj, robot_instance in GameLogic.robotDict.items():
			del obj

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

		# TODO: Reimplement the restart function,
		#  by killing the objects created, and then
		#  calling the init function again.

		# ALL THE FOLLOWING DOES NOT WORK

		for obj, robot_instance in GameLogic.robotDict.items():
			del robot_instance

		for obj, component_instance in GameLogic.componentDict.items():
			del component_instance

		init(contr)

		print ('######### RESTARTING SIMULATION ########')
