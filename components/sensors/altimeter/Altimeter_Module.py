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
		print '######## ALTIMETER INITIALIZATION ########'
		port_name = '{0}/{1}'.format(parent.name, ob['Component_Type'])
		print "OPENING PORTS '{0}'".format(port_name)
		
		GameLogic.orsConnector.registerBufferedPortBottle([port_name])
		#GameLogic.orsConnector.printOpenPorts()
		
		print '######## ALTIMETER INITIALIZED ########'


def output(contr):

	if ob['Init_OK']:	

		state_dict = GameLogic.componentDict[ob]
		
		############### Altitude ###################
		if GameLogic.orsCommunicationEnabled:

		# Compute the altitude : distance between the helicopter
		#  and the first object on the -Z axis (within a range of 5000 m). 
		
			hitObj, hitPoint, hitNormal = ob.rayCast([0,0,-5000], None)
			if hitObj:
				altitude = ob.getDistanceTo(hitPoint)

				if GameLogic.orsCommunicationEnabled:
					p = GameLogic.orsConnector.getPort(port_name)
					bottle = p.prepare()
					bottle.clear()
					bottle.addDouble(altitude)
					#...and send it
					p.write()	
			else:
				if GameLogic.orsVerbose:
					print ' No ground -> no altitude!'				
					print "Altitude sent : ",altitude
