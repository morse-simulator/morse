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

	ob['Init_OK'] = False

	try:
		# Get the dictionary for the component's state
		state_dict = GameLogic.componentDict[ob]
		ob['Init_OK'] = True
	except AttributeError:
		print ("Component Dictionary not found!")
		print ("This component must be part of a scene")
		

	if ob['Init_OK']:
		print ('######## CONTROL INITIALIZATION ########')
		speed_port_name = port_name + '/vxvyvz'
		rotation_port_name = port_name + '/rxryrz'
		#print ("OPENING PORTS '{0}', '{1}'".format(speed_port_name, rotation_port_name))

		GameLogic.orsConnector.registerBufferedPortBottle([speed_port_name])
		GameLogic.orsConnector.registerBufferedPortBottle([rotation_port_name])

		print ('######## CONTROL INITIALIZED ########')
	
	
def move(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)

	speed_port_name = port_name + '/vxvyvz'
	rotation_port_name = port_name + '/rxryrz'

	# Get the dictionary for the robot's state
	robot_state_dict = GameLogic.robotDict[parent]

	if ob['Init_OK']:	

	############################### SPEED #################################

		#retrieve the port we want to write on	
		p = GameLogic.orsConnector.getPort(speed_port_name)
		
		#non-blocking read of the port
		cmd = p.read(False)	

		if cmd!=None:	
			
			#fps = GameLogic.getAverageFrameRate()		
			ticks = GameLogic.getLogicTicRate()

			vx =   cmd.get(0).asDouble()/ticks
			vy =   cmd.get(1).asDouble()/ticks			
			vz = - cmd.get(2).asDouble()/ticks
			
			rx =   cmd.get(3).asDouble()/ticks
			ry =   cmd.get(4).asDouble()/ticks			
			rz = - cmd.get(5).asDouble()/ticks

			msg_act = contr.actuators['Send_update_msg']
			#msg_act.propName = parent.name
			#msg_act.to = parent.name
			msg_act.subject = 'Speed'		
			robot_state_dict['vx'] = vx
			robot_state_dict['vy'] = vy	
			robot_state_dict['vz'] = vz		

			robot_state_dict['rx'] = rx
			robot_state_dict['ry'] = ry	
			robot_state_dict['rz'] = rz		

			contr.activate(msg_act)

			#print ("Motion for robot '{0}'".format(parent.name))
			#print ("\tvx: ",vx," vy: ",vy," vz: ",vz)
			#print ("\trx: ",rx," ry: ",ry," rz: ",rz )
