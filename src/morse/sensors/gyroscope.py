import GameLogic
import morse.helpers.sensor
import morse.helpers.math

class GyroscopeClass(morse.helpers.sensor.MorseSensorClass):
	""" Gyroscope sensor """

	def __init__(self, obj, parent=None):
		""" Constructor method.

		Receives the reference to the Blender object.
		The second parameter should be the name of the object's parent.
		"""
		print ("######## GYROSCOPE '%s' INITIALIZING ########" % obj.name)
		# Call the constructor of the parent class
		super(self.__class__,self).__init__(obj, parent)

		self.local_data['yaw'] = 0.0
		self.local_data['pitch'] = 0.0
		self.local_data['roll'] = 0.0

		self.data_keys = ['yaw', 'pitch', 'roll']

		# Initialise the copy of the data
		for variable in self.data_keys:
			self.modified_data.append(self.local_data[variable])

		print ('######## GYROSCOPE INITIALIZED ########')


	def default_action(self):
		""" Get the yaw, pitch and roll of the blender object. """
		yaw = self.position_3d.yaw
		pitch = self.position_3d.pitch
		roll = self.position_3d.roll
		#yaw, pitch, roll = morse.helpers.math.euler_angle(self.blender_obj)

		# Store the values in the robot's object
		self.robot_parent.yaw = yaw
		self.robot_parent.pitch = pitch
		self.robot_parent.roll = roll

		# Store the data acquired by this sensor that could be sent
		#  via a middleware.
		self.local_data['yaw'] = float(yaw)
		self.local_data['pitch'] = float(pitch)
		self.local_data['roll'] = float(roll)
