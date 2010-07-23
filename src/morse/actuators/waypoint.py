import GameLogic
import Mathutils
import morse.helpers.actuator

class WaypointActuatorClass(morse.helpers.actuator.MorseActuatorClass):
	""" Waypoint motion controller

	This controller will receive a destination point and
	make the robot move to that location by moving forward and turning.
	This controller is meant for land robots that can not move sideways.
	"""

	def __init__(self, obj, parent=None):

		print ('######## CONTROL INITIALIZATION ########')
		# Call the constructor of the parent class
		super(self.__class__,self).__init__(obj, parent)

		self.tolerance = 0.5
		self.angle_tolerance = 5
		self.destination = self.blender_obj.position

		# Direction of the global vectors
		self.world_X_vector = Mathutils.Vector([1,0,0])
		self.world_Y_vector = Mathutils.Vector([0,1,0])

		self.local_data['x'] = self.destination[0]
		self.local_data['y'] = self.destination[1]
		self.local_data['z'] = self.destination[2]
		self.local_data['speed'] = 5.0

		self.data_keys = ['x', 'y', 'z', 'speed']

		# Initialise the copy of the data
		for variable in self.data_keys:
			self.modified_data.append(self.local_data[variable])

		print ('######## CONTROL INITIALIZED ########')


	def default_action(self):
		""" Move the object towards the destination. """
		parent = self.robot_parent
		speed = self.local_data['speed']
		vx = 0
		rz = 0

		self.destination = [ self.local_data['x'], self.local_data['y'], self.local_data['z'] ]

		#print ("WAYPOINT GOT DESTINATION: {0}".format(self.destination))
		#print ("Robot {0} move status: '{1}'".format(parent.blender_obj.name, parent.move_status))

		# Vectors returned are already normalised
		distance, global_vector, local_vector = self.blender_obj.getVectTo(self.destination)
		# Convert to the Blender Vector object
		global_vector = Mathutils.Vector(global_vector)

		#print ("My position: {0}".format(self.blender_obj.position))
		print ("\nGOT DISTANCE: {0}".format(distance))
		print ("Global vector: {0}".format(global_vector))
		#print ("Local  vector: {0}".format(local_vector))

		# If the target has been reached, change the status
		if distance-self.tolerance <= 0:
			parent.move_status = "Stop"
			#print ("TARGET REACHED")
			#print ("Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus']))

		else:
			parent.move_status = "Transit"

			# Use the appropriate function to get the angle between two vectors
			if GameLogic.pythonVersion < 3:
				target_angle = Mathutils.AngleBetweenVecs(global_vector, self.world_X_vector)
			else:
				# In Blender 2.5, the angle function returns radians
				target_angle = global_vector.angle(self.world_X_vector)
				# Convert to degrees
				target_angle = target_angle * 180 / math.pi

			"""
			# Correct the direction of the turn according to the angles
			dot = global_vector.dot(self.world_Y_vector)
			print ("DOT = {0}".format(dot))
			if dot < 0:
				target_angle = target_angle * -1
			"""

			try:
				robot_angle = parent.yaw
			except KeyError as detail:
				print ("Gyroscope angle not found. Does the robot have a Gyroscope?")
				print (detail)
				# Force the robot to move towards the target, without rotating
				robot_angle = target_angle

			# Get the direction difference between the robot and the target
			angle_diff = robot_angle - target_angle
			"""
			if target_angle < robot_angle:
				angle_diff = robot_angle - target_angle
				rotation_direction = -1
			else:
				angle_diff = target_angle - robot_angle
				rotation_direction = 1

			# Make a correction when the angles change signs
			if angle_diff > 180:
				angle_diff = 360 - angle_diff
				rotation_direction = rotation_direction * -1
			"""
			rotation_direction = -1

			print ("Angles: R=%.4f, T=%.4f  Diff=%.4f  Direction = %d" % (robot_angle, target_angle, angle_diff, rotation_direction))

			# Tick rate is the real measure of time in Blender.
			# By default it is set to 60, regardles of the FPS
			# If logic tick rate is 60, then: 1 second = 60 ticks
			ticks = GameLogic.getLogicTicRate()
			try:
				# Move forward
				vx = speed / ticks
				# Test if the orientation of the robot is within tolerance
				if -self.angle_tolerance < angle_diff < self.angle_tolerance:
					rz = 0
				# If not, rotate the robot
				else:
					rz = ((speed / ticks) / 2) * rotation_direction
			# For the moment ignoring the division by zero
			# It happens apparently when the simulation starts
			except ZeroDivisionError:
				pass

		
		# Give the movement instructions directly to the parent
		# The second parameter specifies a "local" movement
		parent.blender_obj.applyMovement([vx, 0, 0], True)
		parent.blender_obj.applyRotation([0, 0, rz], True)
