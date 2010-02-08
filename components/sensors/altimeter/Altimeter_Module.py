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
		print '######## ALTIMETER INITIALIZATION ########'
		print "OPENING PORTS '{0}'".format(port_name)
		GameLogic.orsConnector.registerBufferedPortBottle([port_name])
		#GameLogic.orsConnector.printOpenPorts()
		print '######## ALTIMETER INITIALIZED ########'


def output(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)

	if ob['Init_OK']:	

		state_dict = GameLogic.componentDict[ob]
		
		############### Altitude ###################
		if GameLogic.orsCommunicationEnabled:

		# Compute the altitude : distance between the helicopter
		#  and the first object on the -Z axis (within a range of 5000 m). 
		
			hitObj, hitPoint, hitNormal = ob.rayCast([0,0,-5000], None)
			if hitObj:
				altitude = ob.getDistanceTo(hitPoint)

				# Define the message structure to send.
				# It is a list of tuples (data, type).
				message_data = [ (altitude, 'double') ]
				GameLogic.orsConnector.postMessage(message_data, port_name)

			else:
				if GameLogic.orsVerbose:
					print ' No ground -> no altitude!'				
					print "Altitude sent : ",altitude
