import math
import GameLogic
import MorseObject
import MorseMath

class ThermometerClass(MorseObject.MorseObjectClass):
	""" Class definition for the gyroscope sensor.
		Sub class of Morse_Object. """
	temperature = 0.0
	global_temp = 15.0
	fire_temp = 200.0

	def __init__(self, obj, parent=None):
		""" Constructor method.
			Receives the reference to the Blender object.
			The second parameter should be the name of the object's parent. """
		print ("######## THERMOMETER '%s' INITIALIZING ########" % obj.name)
		# Call the constructor of the parent class
		super(self.__class__,self).__init__(obj, parent)

		# Get the global coordinates of defined in the scene
		scene = GameLogic.getCurrentScene()
		script_empty_name = 'Scene_Script_Holder'
		# Prefix the name of the component with 'OB'
		# Will only be necessary until the change to Blender 2.5
		if GameLogic.pythonVersion < 3:
			script_empty_name = 'OB' + script_empty_name
		script_empty = scene.objects[script_empty_name]
		self.global_temp = float(script_empty['Temperature'])

		print ('######## THERMOMETER INITIALIZED ########')


	def default_action(self):
		""" Compute the local temperature

		Temperature is measured dependent on the closest fire source.
		"""
		max_distance = 0.0
		fires = False

		scene = GameLogic.getCurrentScene()
		# Look for the fire sources marked so
		for obj in scene.objects:
			try:
				obj['Fire']
				fire_radius = obj['Fire_Radius']
				# If the direction of the fire is also important,
				#  we can use getVectTo instead of getDistanceTo
				distance = self.blender_obj.getDistanceTo(obj)
				if distance > max_distance:
					max_distance = distance
					fires = True
			except KeyError as detail:
				# print "Exception: ", detail
				pass

		self.temperature = self.global_temp
		# Trial and error formula to get a temperature dependant on
		#  distance. 
		if fires:
			self.temperature += self.fire_temp * math.e ** (-0.2 * max_distance)

		# Store the data acquired by this sensor that could be sent
		#  via a middleware.
		# It is a list of tuples (name, data, type).
		self.message_data = [ ('Temperature', self.temperature, 'double') ]

		#print ("[Temperature %.4f" % (self.temperature))
