import GameLogic
import Mathutils
from collections import deque
import morse.helpers.object

class WaypointActuatorClass(morse.helpers.object.MORSEObjectClass):
	""" Waypoint motion controller

	This controller will receive a destination point and
	make the robot move to that location by moving forward.
	This controller is meant for land robots
	that can not move sideways.
	"""

	def __init__(self, obj, parent=None):

		print ('######## CONTROL INITIALIZATION ########')
		# Call the constructor of the parent class
		super(self.__class__,self).__init__(obj, parent)

		self.speed = 0.0
		self.angle_tolerance = 5.0	# Angles in degrees
		self.waypoint = []
		self.wp_tolerance = 1.0
		self.wp_list = deque()
		self.wp_index = 0

		init_target_object(obj)

		print ('######## CONTROL INITIALIZED ########')

	def init_target_object(obj)
		""" Setup the object that will be used as the target
			for the robot movement (the target the robot will follow). """
		scene = GameLogic.getCurrentScene()
		# Prefix the name of the component with 'OB'
		# Will only be necessary until the change to Blender 2.5
		if GameLogic.pythonVersion < 3:
			obj['TargetObject'] = 'OB' + obj['TargetObject']
		self.target_ob = scene.objects[obj['TargetObject']]
		self.target_ob.setVisible(obj['Show_Target'])


	def default_action(contr):
		""" Main function of this component.
			It will provide a movement and rotation speed to the robot,
			in the form of (v, w). """

		robot_state_dict['moveStatus'] = "Transit"
		print ("WAYPOINT GOT DESTINATION: {0}".format(destination))
		print ("Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus']))
		robot_state_dict['destination'] = destination

		# Reset movement variables
		vx, vy, vz = 0.0, 0.0, 0.0
		rx, ry, rz = 0.0, 0.0, 0.0

		# DEBUGGING:
		# Translate the marker to the target destination
		scene = GameLogic.getCurrentScene()
		target_ob = scene.objects[ob['TargetObject']]
		destination[2] = 0
		target_ob.position = destination

		try:
			# Exit the function if there has been no command to move
			if not robot_state_dict['moveStatus'] == "Transit":
				return
		except KeyError:
			# Also exit if there is no moveStatus property
			return

		scene = GameLogic.getCurrentScene()
		target_ob = scene.objects[ob['TargetObject']]
		destination = target_ob.position
		# Ignore the altitude (Z)
		destination[2] = 0

		# Calculate the direction needed
		location_V = Mathutils.Vector(ob.position)
		# Ignore the altitude (Z)
		location_V[2] = 0
		destination_V = Mathutils.Vector(destination)

		distance_V = destination_V - location_V
		distance = distance_V.length - robot_state_dict['tolerance']

		#print ("GOT DISTANCE: {0}".format(distance))

		world_X_vector = Mathutils.Vector([1,0,0])
		world_Y_vector = Mathutils.Vector([0,1,0])
		distance_V.normalize()
		# Use the appropriate function to get the angle between two vectors
		if GameLogic.pythonVersion < 3:
			target_angle = Mathutils.AngleBetweenVecs(distance_V, world_X_vector)
		else:
			# In Blender 2.5, the angle function returns radians
			target_angle = distance_V.angle(world_X_vector)
			# Convert to degrees
			target_angle = target_angle * 180 / math.pi

		# Correct the direction of the turn according to the angles
		dot = distance_V.dot(world_Y_vector)
		if dot < 0:
			target_angle = target_angle * -1

		try:
			robot_angle = robot_state_dict['Yaw']
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
			# Move forward
			vx = robot_state_dict['speed']
			# Test if the orientation of the robot is within tolerance
			# If not, rotate the robot
			if not (-angle_tolerance < angle_diff and angle_diff < angle_tolerance):
				#rz = 0.03 * rotation_direction
				rz = (robot_state_dict['speed'] / 2) * rotation_direction
		# If the target has been reached, change the status
		elif distance <= 0:
			robot_state_dict['moveStatus'] = "Stop"
			print ("TARGET REACHED")
			print ("Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus']))

		# Give the movement instructions directly to the parent
		# The second parameter specifies a "local" movement
		self.robot_parent.applyMovement([vx, vy, vz], True)
		self.robot_parent.applyRotation([rx, ry, rz], True)

		#print ("Motion for robot '{0}'".format(parent.name))
		#print ("\tvx: %.4f, %4f, %4f" % (vx, vy, vz))
		#print ("\trx: %.4f, %4f, %4f" % (rx, ry, rz))
