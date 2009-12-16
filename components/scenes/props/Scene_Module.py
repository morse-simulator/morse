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
			# pass
			sys.exc_clear()  # Clears the last exception thrown

	# Get the components
	for obj, robot_state_dict in GameLogic.robotDict.iteritems():
		# Create an empty list for the components of this robot
		component_list = []
		for child in obj.childrenRecursive:
			try:
				# Look for the components tagged as such
				child['Component_Tag']
				component_list.append (child['Component_Type'])
				robot_state_dict['components'] = component_list

				# Create an empty dictionary for each component,
				#  and add it to GameLogic
				GameLogic.componentDict[child] = {}
			except KeyError:
				# pass
				sys.exc_clear()  # Clears the last exception thrown
		#print ("GameLogic[{0}] = {1}".format(name, obj))


# Print the contents of the robot and component dictionaries
def Check_Dictionaries():
	print ("------------------------------------")
	print ("GameLogic has the following robots:")
	for obj, robot_state_dict in GameLogic.robotDict.iteritems():
		print ("\tROBOT: '{0}'".format(obj))
		for component in robot_state_dict['components']:
			print ("\t\t- Component: '{0}'".format(component))

	print ("GameLogic has the following components:")
	for obj, component_variables in GameLogic.componentDict.iteritems():
		print ("\tCOMPONENT: '{0}'".format(obj))


# Create a port that publishes the list of components
# This will consist of 3 nested bottles:
##  1 Contains all the robots
##	2 Contains the name and the list of components of a robot
##  3 The list of components
def Publish_Dictionaries(port_name):

	if GameLogic.orsCommunicationEnabled:
		p = GameLogic.orsConnector.getPort(port_name)
		bottle = p.prepare()
		bottle.clear()

		# Create a structure of nested bottles
		"""
		for obj, robot_state_dict in GameLogic.robotDict.iteritems():
			bottle2 = bottle.addList()
			bottle2.addString(obj.name)
			bottle3 = bottle2.addList()
			for component in robot_state_dict['components']:
				bottle3.addString(component.name)
		"""

		# Serialize the lists using JSON
		scene_elems = []
		for obj, robot_state_dict in GameLogic.robotDict.iteritems():
			robot_list = [obj.name, robot_state_dict['components']]
			scene_elems.append (robot_list)
		message = json.dumps(scene_elems)

		# print ("LIST: ", scene_elems)
		# print ("JSON: ", message)
		# print ("BACK: ", json.loads(message))

		bottle.fromString(message)

		#...and send it
		p.write()



def init(contr):
	print ('######## SCENE INITIALIZATION ########')
	print
	print ("Scripts path: " + scriptRoot)
	print ("Lib path: " + libRoot)
	print

	# Middleware initialization
	#if not hasattr(GameLogic, 'orsConnector'):
	GameLogic.orsConnector = MiddlewareConnector()

	GameLogic.orsCommunicationEnabled = True

	print ('======== COMPONENT DICTIONARY INITIALIZATION =======')
	Create_Dictionaries()

	print ("OPENING PORT '{0}'".format(in_port_name))
	print ("OPENING PORT '{0}'".format(out_port_name))
	GameLogic.orsConnector.registerBufferedPortBottle([in_port_name, out_port_name])
	print ('======= COMPONENT DICTIONARY INITIALIZED =======')

	Check_Dictionaries()

	Publish_Dictionaries(out_port_name)

def admin(contr):

	#retrieve the port we want to write on
	p_in = GameLogic.orsConnector.getPort(in_port_name)

	#non-blocking read of the port
	command_data = p_in.read(False)

	if command_data != None:
		command = command_data.get(0).toString()

		print ("Data:\t\t'{0}'".format(command_data))
		print ("Command:\t'{0}'".format(command_data.get(0).toString()))

		print (" ===>> Communication with external agent established!")

		if command == "list_robots":
			print (" ===>> Sending list of elements")
			Publish_Dictionaries(out_port_name)


def finish(contr):
	"""Close the open ports."""

	sensor = contr.sensors['ESC_KEY']

	#execute only when the ESC key is released (if we don't test that,
	#the code get executed two time, when pressed, and when released)
	if not sensor.positive and sensor.triggered:
		print ('######### FINALIZING... ########')

		GameLogic.orsConnector.finalize()

		quitActuator = contr.actuators['Quit_sim']
		contr.activate(quitActuator)

		print ('######### EXITING SIMULATION ########')
