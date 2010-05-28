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
		position_port_name = port_name + '/position'
		angle_port_name = port_name + '/angle'
		print ("OPENING PORTS '{0}', '{1}'".format(position_port_name, angle_port_name))

		GameLogic.orsConnector.registerBufferedPortBottle([position_port_name])
		GameLogic.orsConnector.registerBufferedPortBottle([angle_port_name])

		print ('######## CONTROL INITIALIZED ########')
	
	
def position(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	position_port_name = port_name + '/position'
	angle_port_name = port_name + '/angle'

	# Get the dictionary for the robot's state
	robot_state_dict = GameLogic.robotDict[parent]

	if ob['Init_OK']:	

	############################### POSITION / ANGLE  #################################

		#retrieve the port we want to write on	
		p = GameLogic.orsConnector.getPort(position_port_name)
		
		#non-blocking read of the port
		cmd = p.read(False)		
		if cmd!=None:	
		   	x =    cmd.get(0).asDouble()
			y =    cmd.get(1).asDouble()			
			z =    cmd.get(2).asDouble()
			ax =   cmd.get(3).asDouble()
			ay =   cmd.get(4).asDouble()			
			az =   cmd.get(5).asDouble()

			print ("received position x : "+str(x)+" y: "+str(y)+" z: "+str(z))
			robot_state_dict['x'] = x
			robot_state_dict['y'] = y	
			robot_state_dict['z'] = z	

			robot_state_dict['ax'] = ax
			robot_state_dict['ay'] = ay	
			robot_state_dict['az'] = az
			print ("received rotation x : "+str(ax)+" y: "+str(ay)+" z: "+str(az))
			#ob.position=pos
			
			msg_act = contr.actuators['Send_update_msg']
			msg_act.propName = parent.name
			msg_act.to = parent.name
			msg_act.subject = 'Position'	

			contr.activate(msg_act)
					   
			#fps = GameLogic.getAverageFrameRate()		
			#vx =   cmd.get(0).asDouble()/fps
			#vy =   cmd.get(1).asDouble()/fps			
			#vz = - cmd.get(2).asDouble()/fps
			
			#rx =   cmd.get(3).asDouble()/fps
			#ry =   cmd.get(4).asDouble()/fps			
			#rz = - cmd.get(5).asDouble()/fps

			#msg_act = contr.actuators['Send_update_msg']
			#msg_act.propName = parent.name
			#msg_act.to = parent.name
			#msg_act.subject = 'Position'		
			#robot_state_dict['vx'] = vx
			#robot_state_dict['vy'] = vy	
			#robot_state_dict['vz'] = vz		

			#robot_state_dict['rx'] = rx
			#robot_state_dict['ry'] = ry	
			#robot_state_dict['rz'] = rz		

			#contr.activate(msg_act)

			#print ("Motion for robot '{0}'".format(parent.name))
			#print ("\tvx: ",vx," vy: ",vy," vz: ",vz)
			#print ("\trx: ",rx," ry: ",ry," rz: ",rz )
