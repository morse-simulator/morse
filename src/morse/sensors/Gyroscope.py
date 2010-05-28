import GameLogic
import MorseObject
import MorseMath

class GyroscopeClass(MorseObject.MorseObjectClass):
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

		print ('######## GYROSCOPE INITIALIZED ########')


	def default_action(self):
		""" Get the yaw, pitch and roll of the blender object. """

		yaw, pitch, roll = MorseMath.euler_angle(self.blender_obj)

		# Store the values in the robot's object
		self.robot_parent.yaw = yaw
		self.robot_parent.pitch = pitch
		self.robot_parent.roll = roll

		# Store the data acquired by this sensor that could be sent
		#  via a middleware.
		self.local_data['yaw'] = float(yaw)
		self.local_data['pitch'] = float(pitch)
		self.local_data['roll'] = float(roll)
