import sys, os
import GameLogic
import Mathutils
import math
from middleware.independent.IndependentBlender import *
import setup.ObjectData



def move(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	dest_port_name = port_name + '/in'
	speed_port_name = port_name + '/speed/in'

	# Radius of tolerance for waypoints
	destination = []

	# Direction tolerance for the movement (in degrees)
	angle_tolerance = 5

	# Get the dictionary for the robot's state
	robot_state_dict = GameLogic.robotDict[parent]

	if ob['Init_OK']:

		# Reset movement variables
		vx, vy, vz = 0.0, 0.0, 0.0
		rx, ry, rz = 0.0, 0.0, 0.0

	    ########################### SPEED ###############################
		# Retrieve the port we want to read from
		sp = GameLogic.orsConnector.getPort(speed_port_name)

		#non-blocking read of the port
		speed_msg = sp.read(False)

		if speed_msg!=None:
			robot_state_dict['speed'] = speed_msg.get(0).asDouble()
			print ("SETTING SPEED TO: {0}".format(robot_state_dict['speed']))


	    ########################### DESTINATION ###########################
		# Retrieve the port we want to read from
		p = GameLogic.orsConnector.getPort(dest_port_name)

		#non-blocking read of the port
		dest = p.read(False)

		if dest!=None:
			for i in range(3):
				destination.append( dest.get(i).asDouble() )

			robot_state_dict['moveStatus'] = "Transit"
			print ("WAYPOINT GOT DESTINATION: {0}".format(destination))
			print ("Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus']))
			robot_state_dict['destination'] = destination

			# DEBUGGING:
			# Translate the marker to the target destination
			scene = GameLogic.getCurrentScene()
			target_ob = scene.objects[ob['TargetObject']]
			destination[2] = 0
			target_ob.position = destination
			try:
				area_ob = scene.objects['OBWP_Area']
				area_ob.scaling = (robot_state_dict['tolerance'], robot_state_dict['tolerance'], 1)
			except KeyError:
				pass

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
			robot_angle = robot_state_dict['gyro_angle']
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

		msg_act = contr.actuators['Send_update_msg']
		msg_act.propName = parent.name
		msg_act.subject = 'Speed'
		robot_state_dict['vx'] = vx
		robot_state_dict['vy'] = vy
		robot_state_dict['vz'] = vz

		robot_state_dict['rx'] = rx
		robot_state_dict['ry'] = ry
		robot_state_dict['rz'] = rz

		contr.activate(msg_act)

		#print ("Motion for robot '{0}'".format(parent.name))
		#print ("\tvx: %.4f, %4f, %4f" % (vx, vy, vz))
		#print ("\trx: %.4f, %4f, %4f" % (rx, ry, rz))
