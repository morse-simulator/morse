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

# Definition of global variables
ob = ''
port_name = ''


def init(contr):
	global ob
	global port_name

	# Middleware initialization
	if not hasattr(GameLogic, 'orsConnector'):
		GameLogic.orsConnector = MiddlewareConnector()

	# To get the game object this controller is on:
	ob = contr.owner
	parent = ob.parent

	# If there is no parent (when testing individual component)
	#  set this component as its own parent
	if not parent:
		parent = ob

	ob['Init_OK'] = False

	try:
		# Get the dictionary for the component's state
		state_dict = GameLogic.componentDict[ob]
		ob['Init_OK'] = True
	except AttributeError:
		print "Component Dictionary not found!"
		print "This component must be part of a scene"

	if ob['Init_OK']:
		port_name = '{0}/{1}'.format(parent.name, ob['Component_Type'])
		
		print '######## GPS INITIALIZATION ########'
		print "OPENING PORTS '{0}'".format(port_name)
		GameLogic.orsConnector.registerBufferedPortBottle([port_name])
		print '######## GPS INITIALIZED ########'



def output(contr):
	global ob
	global port_name

	if ob['Init_OK']:	

		if GameLogic.orsCommunicationEnabled:
			position=ob.position	
			x=position[0];
			y=position[1];
			z=-position[2];
			
			p = GameLogic.orsConnector.getPort(port_name)	
			bottle = p.prepare()
			bottle.clear()
			bottle.addDouble(x)
			bottle.addDouble(y)
			bottle.addDouble(z)			
			#...and send it
			p.write()
			
			print "GPS NED sent  x: ",x," y: ",y," z:", z
