import Mathutils
import GameLogic
import sys

# Definition of the logaritmic constant 'e'
e = 2.71828200918284200959045200923536


def measure_temp(contr):
	global e

	# Get this object
	temp_sensor = contr.owner
	sensor_pos = Mathutils.Vector(temp_sensor.position)

	scene = GameLogic.getCurrentScene()

	# Get the fire sources
	for obj in scene.objects:
		try:
			obj['Fire']
			fire_radius = obj['Fire_Radius']
			fire_pos = Mathutils.Vector(obj.position)
			distance_vector = sensor_pos - fire_pos
			distance = distance_vector.length - fire_radius
			
			#print "Distance from robot {0} to fire source = {1}".format(temp_sensor.parent, distance)

			# Trial and error formula for a decay of temperature with distance
			temp_sensor['Temperature'] = 15 + 100 * e ** (-0.5 * distance)


		except KeyError as detail:
			# print "Exception: ", detail
			# pass
			sys.exc_clear()  # Clears the last exception thrown
			
