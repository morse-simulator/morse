import GameLogic
import MorseObject
import MorseMath

class GPSClass(MorseObject.MorseObjectClass):
	""" Class definition for the gyroscope sensor.
		Sub class of Morse_Object. """
	x = 0.0
	y = 0.0
	z = 0.0
	global_x = 0.0
	global_y = 0.0
	global_z = 0.0

	def __init__(self, obj, parent=None):
		""" Constructor method.
			Receives the reference to the Blender object.
			The second parameter should be the name of the object's parent. """
		print ("######## GPS '%s' INITIALIZING ########" % obj.name)
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
		self.global_x = float(script_empty['UTMXOffset'])
		self.global_y = float(script_empty['UTMYOffset'])
		self.global_z = float(script_empty['AltitudeOffset'])

		print ('######## GPS INITIALIZED ########')


	def default_action(self):
		""" Main function of this component. """

		# Get the coordinates of the object, and correct them
		#  using the global settings of the scene
		self.x = self.global_x + self.blender_obj.position[0]
		self.y = self.global_y + self.blender_obj.position[1]
		self.z = self.global_z + self.blender_obj.position[2]

		# Store the data acquired by this sensor that could be sent
		#  via a middleware.
		# It is a list of tuples (name, data, type).
		self.message_data = [ ('x', self.x, 'double'), ('y', self.y, 'double'), ('z', self.z, 'double') ]

		#print ("[Y %.4f, P %.4f, R %.4f" % (self.x, self.y, self.z))
