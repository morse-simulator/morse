import GameLogic
import Mathutils
import morse.helpers.actuator

class WaypointActuatorClass(morse.helpers.actuator.MORSEActuatorClass):
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
		self.destination = self.blender_obj.position

		# Direction of the global vectors
		self.world_X_vector = Mathutils.Vector([1,0,0])
		self.world_Y_vector = Mathutils.Vector([0,1,0])

		self.local_data['speed'] = 5.0
		self.local_data['x'] = self.destination[0]
		self.local_data['y'] = self.destination[1]
		self.local_data['z'] = self.destination[2]

		self.data_keys = ['x', 'y', 'z']

		# Initialise the copy of the data
		for variable in self.data_keys:
			self.modified_data.append(self.local_data[variable])

		print ('######## CONTROL INITIALIZED ########')


	def default_action(self):
		""" Move the object towards the destination. """
		parent = self.robot_parent
		self.speed = self.local_data['speed']

		self.destination = [ self.local_data['x'], self.local_data['y'], self.local_data['z'] ]

		#print ("WAYPOINT GOT DESTINATION: {0}".format(self.destination))
		#print ("Robot {0} move status: '{1}'".format(parent.blender_obj.name, parent.move_status))

		# Vectors returned are already normalised
		distance, global_vector, local_vector = self.blender_obj.getVectTo(self.destination)

		#print ("My position: {0}".format(self.blender_obj.position))
		#print ("GOT DISTANCE: {0}".format(distance))
		#print ("Global vector: {0}".format(global_vector))
		#print ("Local  vector: {0}".format(local_vector))

		# Use the appropriate function to get the angle between two vectors
		if GameLogic.pythonVersion < 3:
			target_angle = Mathutils.AngleBetweenVecs(global_vector, self.world_X_vector)
		else:
			# In Blender 2.5, the angle function returns radians
			target_angle = global_vector.angle(self.world_X_vector)
			# Convert to degrees
			target_angle = target_angle * 180 / math.pi

		# Correct the direction of the turn according to the angles
		dot = global_vector.dot(self.world_Y_vector)
		if dot < 0:
			target_angle = target_angle * -1

		try:
			robot_angle = robot_parent.yaw
		except KeyError as detail:
			print ("Gyroscope angle not found. Does the robot have a Gyroscope?")
			print (detail)
			# Force the robot to move towards the target, without rotating
			robot_angle = target_angle

		# Get the direction difference between the robot and the target
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

		#print ("Angles: R=%.4f, T=%.4f  Diff=%.4f  Direction = %d" % (robot_angle, target_angle, angle_diff, rotation_direction))

		if distance > 0:
			# Tick rate is the real measure of time in Blender.
			# By default it is set to 60, regardles of the FPS
			# If logic tick rate is 60, then: 1 second = 60 ticks
			ticks = GameLogic.getLogicTicRate()
			try:
				# Move forward
				vx = self.speed / ticks
				# Test if the orientation of the robot is within tolerance
				# If not, rotate the robot
				if not (-angle_tolerance < angle_diff and angle_diff < angle_tolerance):
					#rz = 0.03 * rotation_direction
					rz = ((self.speed / ticks) / 2) * rotation_direction
			# For the moment ignoring the division by zero
			# It happens apparently when the simulation starts
			except ZeroDivisionError:
				pass

		# If the target has been reached, change the status
		elif distance <= 0:
			parent.move_status = "Stop"
			#print ("TARGET REACHED")
			#print ("Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus']))

		# Give the movement instructions directly to the parent
		# The second parameter specifies a "local" movement
		self.robot_parent.applyMovement([vx, vy, vz], True)
		self.robot_parent.applyRotation([rx, ry, rz], True)
