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
		print '######## GPS INITIALIZATION ########'
		print "OPENING PORTS '{0}'".format(port_name)
		GameLogic.orsConnector.registerBufferedPortBottle([port_name])
		print '######## GPS INITIALIZED ########'



def output(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)

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
			
			#print "GPS NED ", ob.name, " sent  x: ",x," y: ",y," z:", z
			ob['Coordinates'] = "%.3f, %.3f, %.3f" % (x, y, z)
