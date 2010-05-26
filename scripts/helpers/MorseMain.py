import sys
import re
import json
import GameLogic

# Import this library to recover the Python version
import platform

# The file Component_Config.py is at the moment included
#  in the .blend file of the scene
import Component_Config


# Global variables for the names of the input and output ports
in_port_name = "admin/in"
out_port_name = "admin/out"


# Create a list of the robots in the scene
def create_dictionaries ():
	"""Creation of a list of all the robots and components in the scene.
	   Uses the properties of the objects to determine what they are."""

	# Create a dictioary of the components in the scene
	if not hasattr(GameLogic, 'componentDict'):
		GameLogic.componentDict = {}

	# Create a dictionary of the robots in the scene
	if not hasattr(GameLogic, 'robotDict'):
		GameLogic.robotDict = {}

	# Create a dictionary with the modifiers
	if not hasattr(GameLogic, 'modifierDict'):
		GameLogic.modifierDict = {}

	# Create a dictionary with the middlewares used
	if not hasattr(GameLogic, 'mwDict'):
		GameLogic.mwDict = {}

	scene = GameLogic.getCurrentScene()

	# Get the robots
	for obj in scene.objects:
		try:
			obj['Robot_Tag']
			# Create an object instance and store it
			instance = create_instance (obj)
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
				instance = create_instance (child, robot_instance)
				GameLogic.componentDict[child.name] = instance

			except KeyError:
				pass
				#sys.exc_clear()	# Clears the last exception thrown
									# Does not work in Python 3

	# Get the middlewares
	for obj in scene.objects:
		try:
			obj['Modifier_Tag']
			# Create an object instance and store it
			instance = create_instance (obj)
			GameLogic.modifierDict[obj] = instance
		except KeyError:
			pass
			#sys.exc_clear()	# Clears the last exception thrown
								# Does not work in Python 3


	# Get the middlewares
	for obj in scene.objects:
		try:
			obj['Middleware_Tag']
			# Create an object instance and store it
			instance = create_instance (obj)
			GameLogic.mwDict[obj] = instance
		except KeyError:
			pass
			#sys.exc_clear()	# Clears the last exception thrown
								# Does not work in Python 3




def check_dictionaries():
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

	print ("\nGameLogic has the following modifiers:")
	for obj, modifier_variables in GameLogic.modifierDict.items():
		print ("\tMODIFIER: '{0}'".format(obj))

	print ("\nGameLogic has the following middlewares:")
	for obj, mw_variables in GameLogic.mwDict.items():
		print ("\tMIDDLEWARE: '{0}'".format(obj))



def create_instance(obj, parent=None):
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
	klass = getattr(module, obj['Class'] + 'Class')
	instance = klass(obj, parent)

	return instance


def link_middlewares():
	""" Read the configuration script (inside the .blend file)
		and assign the correct middleware and options to each component. """
	# Add the hook functions to the appropriate components
	component_list = Component_Config.component_mw
	for component_name, mw_data in component_list.items():
		(mw_name, mw_io, mw_function) = mw_data
		# Prefix the name of the component with 'OB'
		# Will only be necessary until the change to Blender 2.5
		if GameLogic.pythonVersion < 3:
			component_name = 'OB' + component_name

		print ("Component: '%s' operated by '%s' as %sput" % (component_name, mw_name, mw_io))
		# Look for the listed mw in the dictionary of active mw's
		for mw_obj, mw_instance in GameLogic.mwDict.items():
			if mw_name in mw_obj.name:
				# Get the instance of the object
				try:
					instance = GameLogic.componentDict[component_name]
				except KeyError as detail:
					print ("Component listed in Component_Config.py not found in scene: {0}".format(detail))
					continue

				if mw_io == 'out':
					# Make the middleware object take note of the component
					mw_instance.register_component(component_name, instance, mw_io)
					# Add the yarp function to the component's action list
					function = getattr(mw_instance, mw_function)
					instance.output_functions.append(function)

				elif mw_io == 'in':
					# Make the middleware object take note of the component
					mw_instance.register_component(component_name, instance, mw_io)
					# Add the yarp function to the component's action list
					function = getattr(mw_instance, mw_function)
					instance.input_functions.append(function)


def add_modifiers():
	""" Read the configuration script (inside the .blend file)
		and assign the correct data modifiers to each component. """
	# Add the hook functions to the appropriate components
	component_list = Component_Config.component_modifier
	for component_name, modifier_name in component_list.items():
		# Prefix the name of the component with 'OB'
		# Will only be necessary until the change to Blender 2.5
		if GameLogic.pythonVersion < 3:
			component_name = 'OB' + component_name

		print ("Component: '%s' operated by '%s'" % (component_name, modifier_name))
		# Look for the listed modifier in the dictionary of active modifier's
		for modifier_obj, modifier_instance in GameLogic.modifierDict.items():
			if modifier_name in modifier_obj.name:
				# Get the instance of the object
				instance = GameLogic.componentDict[component_name]
				# Add the modifier function to the component's action list
				instance.modifier_functions.append(modifier_instance.json_serialise)



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
	create_dictionaries()
	add_modifiers()
	link_middlewares()

	#print ("OPENING PORT '{0}'".format(in_port_name))
	#print ("OPENING PORT '{0}'".format(out_port_name))
	#GameLogic.orsConnector.registerBufferedPortBottle([in_port_name, out_port_name])
	print ('======= COMPONENT DICTIONARY INITIALIZED =======')

	check_dictionaries()



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

		# Force the deletion of the middleware objects
		for obj, mw_instance in GameLogic.mwDict.items():
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

		for obj, component_instance in GameLogic.componentDict.items():
			del component_instance

		for obj, robot_instance in GameLogic.robotDict.items():
			del robot_instance

		# Force the deletion of the middleware objects
		for obj, mw_instance in GameLogic.mwDict.items():
			del obj

		init(contr)

		print ('######### RESTARTING SIMULATION ########')
