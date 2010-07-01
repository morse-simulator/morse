import GameLogic
import morse.helpers.object

class GPSClass(morse.helpers.object.MorseObjectClass):
	""" Class definition for the gyroscope sensor.
		Sub class of Morse_Object. """

	def __init__(self, obj, parent=None):
		""" Constructor method.
			Receives the reference to the Blender object.
			The second parameter should be the name of the object's parent. """
		print ("######## GPS '%s' INITIALIZING ########" % obj.name)
		# Call the constructor of the parent class
		super(self.__class__,self).__init__(obj, parent)

		self.local_data['x'] = 0.0
		self.local_data['y'] = 0.0
		self.local_data['z'] = 0.0
		self._global_x = 0.0
		self._global_y = 0.0
		self._global_z = 0.0

		# Get the global coordinates of defined in the scene
		scene = GameLogic.getCurrentScene()
		script_empty_name = 'Scene_Script_Holder'
		# Prefix the name of the component with 'OB'
		# Will only be necessary until the change to Blender 2.5
		if GameLogic.pythonVersion < 3:
			script_empty_name = 'OB' + script_empty_name
		script_empty = scene.objects[script_empty_name]
		self._global_x = float(script_empty['UTMXOffset'])
		self._global_y = float(script_empty['UTMYOffset'])
		self._global_z = float(script_empty['UTMZOffset'])

		print ('######## GPS INITIALIZED ########')


	def default_action(self):
		""" Main function of this component. """

		# Get the coordinates of the object, and correct them
		#  using the global settings of the scene
		x = self._global_x + self.blender_obj.position[0]
		y = self._global_y + self.blender_obj.position[1]
		z = self._global_z + self.blender_obj.position[2]

		# Store the data acquired by this sensor that could be sent
		#  via a middleware.
		self.local_data['x'] = float(x)
		self.local_data['y'] = float(y)
		self.local_data['z'] = float(z)
