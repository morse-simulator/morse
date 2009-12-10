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

# Definition of the logaritmic constant 'e'
e = 2.71828200918284200959045200923536

def init(contr):
	# Middleware initialization
	if not hasattr(GameLogic, 'orsConnector'):
		GameLogic.orsConnector = MiddlewareConnector()

	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)

	ob['Init_OK'] = False

	try:
		# Get the dictionary for the component's state
		robot_state_dict = GameLogic.robotDict[parent]
		#state_dict = GameLogic.componentDict[ob]
		ob['Init_OK'] = True
	except AttributeError:
		print ("Component Dictionary not found!")
		print ("This component must be part of a scene")

	if ob['Init_OK']:
		print ('######## THERMOMETER INITIALIZATION ########')
		robot_state_dict['temperature'] = 0.0
		print ("OPENING PORTS '{0}'".format(port_name))
		GameLogic.orsConnector.registerBufferedPortBottle([port_name])
		#GameLogic.orsConnector.printOpenPorts()
		print ('######## THERMOMETER INITIALIZED ########')


def output(contr):
	global e

	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)

	if ob['Init_OK']:	
		robot_state_dict = GameLogic.robotDict[parent]

		############### Thermometer ###################

		# Get this object
		temp_sensor = contr.owner
		sensor_pos = Mathutils.Vector(temp_sensor.position)
		temperature = 0.0

		scene = GameLogic.getCurrentScene()

		# Get the fire sources
		for obj in scene.objects:
			try:
				obj['Fire']
				fire_radius = obj['Fire_Radius']
				fire_pos = Mathutils.Vector(obj.position)
				#print "FIRE AT ", fire_pos
				#print "THERMO AT ", sensor_pos
				distance_vector = sensor_pos - fire_pos
				distance = distance_vector.length - fire_radius
				#print "DISTANCE = ", distance
				
				#print "Distance from robot {0} to fire source = {1}".format(temp_sensor.parent, distance)

				# Trial and error formula for a decay of temperature with distance
				temperature = 15 + 200 * e ** (-0.2 * distance)
				temp_sensor['Temperature'] = temperature


			except KeyError as detail:
				# print "Exception: ", detail
				# pass
				sys.exc_clear()  # Clears the last exception thrown


			if GameLogic.orsCommunicationEnabled:
				p = GameLogic.orsConnector.getPort(port_name)
				bottle = p.prepare()
				bottle.clear()
				bottle.addDouble(temperature)
				#...and send it
				p.write()	
