import sys, os
import GameLogic
import Mathutils
from collections import deque

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
	dest_port_name = port_name + '/in'
	speed_port_name = port_name + '/speed/in'

	ob['Init_OK'] = False

	try:
		# Get the dictionary for the component's state
		robot_state_dict = GameLogic.robotDict[parent]
		ob['Init_OK'] = True
	except AttributeError:
		print ("Component Dictionary not found!")
		print ("This component must be part of a scene")

	if ob['Init_OK']:
		print ('######## CONTROL INITIALIZATION ########')
		#print ("OPENING PORT '{0}'".format(dest_port_name))
		GameLogic.orsConnector.registerBufferedPortBottle([dest_port_name])
		#print ("OPENING PORT '{0}'".format(speed_port_name))
		GameLogic.orsConnector.registerBufferedPortBottle([speed_port_name])

		#### INIT ROBOT DICTIONARY ####
		robot_state_dict['speed'] = 0.05
		robot_state_dict['destination'] = 0
		robot_state_dict['tolerance'] = 1.5
		robot_state_dict['path'] = deque()
		robot_state_dict['wp_index'] = 0

		#### INIT TARGET OBJECT ####
		scene = GameLogic.getCurrentScene()
		# Remove the OB part of the object name, when using Blender 2.5
		# Tested by checking for Python 3
		if GameLogic.pythonVersion >= 3:
			ob['TargetObject'] = ob['TargetObject'][2:]
		target_ob = scene.objects[ob['TargetObject']]
		target_ob.setVisible(ob['Show_Target'])
		try:
			if GameLogic.pythonVersion < 3:
				area_ob = scene.objects['OBWP_Area']
			else:
				area_ob = scene.objects['WP_Area']
			area_ob.setVisible(ob['Show_Target'])

			initial_position = target_ob.position
			initial_position[2] = 0.01
			area_ob.position = initial_position
			area_ob.setParent(target_ob)
		except KeyError:
			print ("Controller WARNING: No area object in scene")

		#### MODE SELECTION ####
		print ("Selecting Motion mode: {0}".format(ob['Operation_Mode']))
		select(contr)

		print ('######## CONTROL INITIALIZED ########')



def select(contr):
	""" Change the Game Engine State to the one
		defined in the 'Operation_Mode' property of the object. """

	ob = contr.owner
	mode = ob['Operation_Mode']

	state_act = contr.actuators['State_Selector']
	state_act.mask = mode
	contr.activate(state_act)
