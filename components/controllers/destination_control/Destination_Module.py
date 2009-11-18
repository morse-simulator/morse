import sys, os
import GameLogic
import Mathutils

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
	dest_port_name = port_name + '/destination'

	ob['Init_OK'] = False

	try:
		# Get the dictionary for the component's state
		state_dict = GameLogic.componentDict[ob]
		ob['Init_OK'] = True
	except AttributeError:
		print "Component Dictionary not found!"
		print "This component must be part of a scene"

	if ob['Init_OK']:
		print '######## CONTROL INITIALIZATION ########'
		print "OPENING PORT '{0}'".format(dest_port_name)
		GameLogic.orsConnector.registerBufferedPortBottle([dest_port_name])
		GameLogic.destination = ob.position
		print '######## CONTROL INITIALIZED ########'
	
	

def move(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	dest_port_name = port_name + '/destination'

	# Radius of tolerance for waypoints
	tolerance = 2
	destination = []

	# Get the dictionary for the robot's state
	robot_state_dict = GameLogic.robotDict[parent]

	if ob['Init_OK']:	

	############################### SPEED #################################
		#retrieve the port we want to write on	
		p = GameLogic.orsConnector.getPort(dest_port_name)
		
		#non-blocking read of the port
		dest = p.read(False)	

		if dest!=None:
			for i in range(3):
				destination.append( dest.get(i).asDouble() )

			print "GOT DESTINATION: ", destination			
			GameLogic.destination = destination

		fps = GameLogic.getAverageFrameRate()
		destination = GameLogic.destination
		# Ignore the altitude (Z)
		destination[2] = 0
			
		# Calculate the direction needed
		location_V = Mathutils.Vector(ob.position)
		# Ignore the altitude (Z)
		location_V[2] = 0
		destination_V = Mathutils.Vector(destination)
		distance_V = destination_V - location_V
		distance = distance_V.length - tolerance

		#print "GOT DISTANCE: ", distance_V
		
		vx, vy, vz = 0.0, 0.0, 0.0		
		rx, ry, rz = 0.0, 0.0, 0.0
		
		if (distance > 0):
			distance_V.normalize()
			vx = destination[0]/fps
			vy = destination[1]/fps
			vz = destination[2]/fps

		msg_act = contr.actuators['Send_update_msg']
		msg_act.propName = parent.name
		msg_act.subject = 'Speed'		
		robot_state_dict['vx'] = vx
		robot_state_dict['vy'] = vy	
		robot_state_dict['vz'] = vz		

		robot_state_dict['rx'] = rx
		robot_state_dict['ry'] = ry	
		robot_state_dict['rz'] = rz		

		contr.activate(msg_act)

		#print "Motion for robot '{0}'".format(parent.name)
		#print "\tvx: ",vx," vy: ",vy," vz: ",vz
		#print "\trx: ",rx," ry: ",ry," rz: ",rz 
