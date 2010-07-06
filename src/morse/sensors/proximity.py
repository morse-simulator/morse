import GameLogic
import morse.helpers.sensor

class ProximitySensorClass(morse.helpers.sensor.MorseSensorClass):
	""" Distance sensor to detect nearby robots """

	def __init__(self, obj, parent=None):
		""" Constructor method.

		Receives the reference to the Blender object.
		The second parameter should be the name of the object's parent.
		"""
		print ("######## PROXIMITY '%s' INITIALIZING ########" % obj.name)
		# Call the constructor of the parent class
		super(self.__class__,self).__init__(obj, parent)

		self.local_data['near_robots'] = {}
		try:
			self.range = self.blender_obj['Range']
		except KeyError:
			# Set a default range of 100m
			self.range = 100
			
		self.data_keys = ['near_robots']

		# Initialise the copy of the data
		for variable in self.data_keys:
			self.modified_data.append(self.local_data[variable])

		print ('######## PROXIMITY INITIALIZED ########')


	def default_action(self):
		""" Create a list of robots within a certain radius of the sensor. """

		self.local_data['near_robots'] = {}

		parent = self.robot_parent.blender_obj

		# Get the fire sources
		for robot in GameLogic.robotDict.keys():
			# Skip distance to self
			if parent != robot:
				distance = self._measure_distance_to_robot (parent, robot)
				if distance <= self.range:
					self.local_data['near_robots'][robot.name] = distance



	def _measure_distance_to_robot(self, own_robot, target_robot):
		""" Compute the distance between two robots

		Parameters are two blender objects
		"""
		distance, globalVector, localVector = own_robot.getVectTo(target_robot)
		#print ("Distance from robot {0} to robot {1} = {2}".format(own_robot, target_robot, distance))
		return distance
