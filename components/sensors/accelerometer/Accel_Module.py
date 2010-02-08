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
		print "Component Dictionary not found!"
		print "This component must be part of a scene"

	if ob['Init_OK']:
		print '######## ACCELEROMETER INITIALIZATION ########'
		state_dict['position']=ob.position
		state_dict['prevPosition']=(0.0, 0.0, 0.0)
		state_dict['velocity']=(0.0, 0.0, 0.0)
		state_dict['acceleration']=(0.0, 0.0, 0.0)

		speed_port_name = port_name + '/vxvyvz'
		accel_port_name = port_name + '/axayaz'
		print "OPENING PORTS '{0}', '{1}'".format(speed_port_name, accel_port_name)

		GameLogic.orsConnector.registerBufferedPortBottle([speed_port_name])
		GameLogic.orsConnector.registerBufferedPortBottle([accel_port_name])

		#GameLogic.orsConnector.printOpenPorts()

		print '######## ACCELEROMETER INITIALIZED ########'


def output(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)


	if ob['Init_OK']:	
		state_dict = GameLogic.componentDict[ob]
		
		############### ACC/vxvyvz ###################
		
		if GameLogic.orsCommunicationEnabled:
			position=ob.position	
			x=position[0];
			y=position[1];
			z=position[2];
		
			state_dict['prevPosition']=state_dict['position']
			state_dict['position']=position

		#	v=obj.getLinearVelocity()
		#	v=obj.getVelocity()
			
			pp=state_dict['prevPosition']
			p=state_dict['position']
			
			fps = GameLogic.getAverageFrameRate()
			
			v={}			
			v[0]=(p[0]-pp[0])*fps
			v[1]=(p[1]-pp[1])*fps
			v[2]=(p[2]-pp[2])*fps
			
			vx=v[0]		
			vy=v[1]
			vz=v[2]
		
			#print "orsConnector = ", GameLogic.orsConnector
			#GameLogic.orsConnector.printOpenPorts()
		
			speed_port_name = port_name + '/vxvyvz'

			# Define the message structure to send.
			# It is a list of tuples (data, type).
			message_data = [ (vx, 'double'), (vy, 'double'), (vz, 'double') ]
			GameLogic.orsConnector.postMessage(message_data, speed_port_name)

			state_dict['prevVelocity']=state_dict['velocity']
			state_dict['velocity']=v
			
			print "Velocity sent  vx: ",vx," vy: ",vy," vz:", vz

		############### ACC/axayaz ###################

		if GameLogic.orsCommunicationEnabled:
			
		#	v=obj.getLinearVelocity()
		#	v=obj.getVelocity()
			
			pv=state_dict['prevVelocity']
			v=state_dict['velocity']
			
			a={}			
			a[0]=(v[0]-pv[0])
			a[1]=(v[1]-pv[1])
			a[2]=(v[2]-pv[2])
			
			ax=a[0]		
			ay=a[1]
			az=a[2]
		
			accel_port_name = port_name + '/axayaz'
		
			# Define the message structure to send.
			# It is a list of tuples (data, type).
			message_data = [ (ax, 'double'), (ay, 'double'), (az, 'double') ]
			GameLogic.orsConnector.postMessage(message_data, accel_port_name)

			state_dict['prevAcceleration']=state_dict['acceleration']
			state_dict['acceleration']=a
			
			print "Acceleration sent  ax: ",ax," ay: ",ay," az:", az
