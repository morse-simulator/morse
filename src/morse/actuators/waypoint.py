import math
import GameLogic
if GameLogic.pythonVersion < 3:
	import Mathutils as mathutils
else:
	import mathutils
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
		# Convert the tolerance to radians
		self.angle_tolerance = 5 * math.pi / 180
		self.destination = self.blender_obj.position
		self.in_motion = False

		# Direction of the global vectors
		self.world_X_vector = mathutils.Vector([1,0,0])
		self.world_Y_vector = mathutils.Vector([0,1,0])

		self.local_data['x'] = self.destination[0]
		self.local_data['y'] = self.destination[1]
		self.local_data['z'] = self.destination[2]
		self.local_data['speed'] = 1.0

		self.data_keys = ['x', 'y', 'z', 'speed']

		# Initialise the copy of the data
		for variable in self.data_keys:
			self.modified_data.append(self.local_data[variable])

		try:
			wp_name = self.blender_obj['Target']
			if GameLogic.pythonVersion < 3:
				wp_name = 'OB' + wp_name
			scene = GameLogic.getCurrentScene()
			self.wp_object = scene.objects[wp_name]
			print ("Using object '%s' to indicate motion target" % wp_name)
		except KeyError as detail:
			self.wp_object = None


		print ('######## CONTROL INITIALIZED ########')


	def default_action(self):
		""" Move the object towards the destination. """
		parent = self.robot_parent
		speed = self.local_data['speed']
		vx = 0
		rz = 0

		self.destination = [ self.local_data['x'], self.local_data['y'], self.local_data['z'] ]

		#print ("Robot {0} move status: '{1}'".format(parent.blender_obj.name, parent.move_status))
		#print ("\nWAYPOINT GOT DESTINATION: {0}".format(self.destination))
		# Place the target marker where the robot should go
		if self.wp_object:
			self.wp_object.position = self.destination

		# Vectors returned are already normalised
		distance, global_vector, local_vector = self.blender_obj.getVectTo(self.destination)
		# Convert to the Blender Vector object
		global_vector = mathutils.Vector(global_vector)

		#print ("GOT DISTANCE: %.4f" % (distance))
		#print ("Global vector: %.4f, %.4f, %.4f" % (global_vector[0], global_vector[1], global_vector[2]))
		#print ("Local  vector: %.4f, %.4f, %.4f" % (global_vector[0], global_vector[1], global_vector[2]))

		# If the target has been reached, change the status
		if distance-self.tolerance <= 0:
			parent.move_status = "Stop"
			#print ("TARGET REACHED")
			#print ("Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus']))

		else:
			parent.move_status = "Transit"

			### Get the angle of the robot ###
			try:
				robot_angle = parent.yaw
			except KeyError as detail:
				print ("Gyroscope angle not found. Does the robot have a Gyroscope?")
				print (detail)
				# Force the robot to move towards the target, without rotating
				robot_angle = target_angle

			if GameLogic.pythonVersion < 3:
				# Convert to radians
				robot_angle = robot_angle * -1 * math.pi / 180 


			### Get the angle to the target ###
			# Use the appropriate function to get the angle between two vectors
			if GameLogic.pythonVersion < 3:
				target_angle = mathutils.AngleBetweenVecs(global_vector, self.world_X_vector)
				# Convert to radians
				target_angle = target_angle * math.pi / 180 
			else:
				# In Blender 2.5, the angle function returns radians
				target_angle = global_vector.angle(self.world_X_vector)

			# Correct the direction of the turn according to the angles
			dot = global_vector.dot(self.world_Y_vector)
			#print ("DOT = {0}".format(dot))
			if dot < 0:
				target_angle = target_angle * -1

			### Get the angle that the robot must turn ###
			if target_angle < robot_angle:
				angle_diff = robot_angle - target_angle
				rotation_direction = -1
			else:
				angle_diff = target_angle - robot_angle
				rotation_direction = 1

			# Make a correction when the angles change signs
			if angle_diff > math.pi:
				angle_diff = (2 * math.pi) - angle_diff
				rotation_direction = rotation_direction * -1

			#print ("Angles: R=%.4f, T=%.4f  Diff=%.4f  Direction = %d" % (robot_angle, target_angle, angle_diff, rotation_direction))

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
