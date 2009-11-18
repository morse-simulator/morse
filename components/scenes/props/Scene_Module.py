import sys, os
import GameLogic

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
				component_list.append (child)
				robot_state_dict['components'] = component_list
				
				# Create an empty dictionary for each component,
				#  and add it to GameLogic
				GameLogic.componentDict[child] = {}
			except KeyError:
				# pass
				sys.exc_clear()  # Clears the last exception thrown
		#print "GameLogic[{0}] = {1}".format(name, obj)
	
	
# Print the contents of the robot and component dictionaries
def Check_Dictionaries():
	print "------------------------------------"
	print "GameLogic has the following robots:"
	for obj, robot_state_dict in GameLogic.robotDict.iteritems():
			print "\tROBOT: '{0}'".format(obj)
			for component in robot_state_dict['components']:
				print "\t\t- Component: '{0}'".format(component)
					
	print "GameLogic has the following components:"
	for obj, component_variables in GameLogic.componentDict.iteritems():
			print "\tCOMPONENT: '{0}'".format(obj)


def init(contr):
	print '######## SCENE INITIALIZATION ########'
	print
	print 'Scripts path: ', scriptRoot
	print 'Lib path: ', libRoot	
	print '\n'

	# Middleware initialization
	#if not hasattr(GameLogic, 'orsConnector'):
	GameLogic.orsConnector = MiddlewareConnector()

	GameLogic.orsCommunicationEnabled = True

	Create_Dictionaries()
	Check_Dictionaries()


def finish(contr):
	"""Close the open ports."""

	sensor = contr.sensors['ESC_KEY']

	#execute only when the ESC key is released (if we don't test that, 
	#the code get executed two time, when pressed, and when released)
	if not sensor.positive and sensor.triggered:
		print '######### FINALIZING... ########'

		#YarpBlender.finalize()
		GameLogic.orsConnector.finalize()

		quitActuator = contr.actuators['Quit_sim']
		contr.activate(quitActuator)

		print '######### EXITING SIMULATION ########'
