import sys, os
import GameLogic
import Mathutils
import json
from collections import deque

try:
	scriptRoot = os.path.join(os.environ['ORS_ROOT'],'scripts')
except KeyError:
	scriptRoot = '.'

try:
	libRoot = os.path.join(os.environ['ORS_ROOT'],'lib')
except KeyError:
	libRoot = '.'

if scriptRoot not in sys.path:
	sys.path.append(scriptRoot)
if scriptRoot not in sys.path:
	sys.path.append(libRoot)

from middleware.independent.IndependentBlender import *
import setup.ObjectData


# Direction tolerance for the movement (in degrees)
angle_tolerance = 5

def init(contr):
	# Middleware initialization
	if not hasattr(GameLogic, 'orsConnector'):
		GameLogic.orsConnector = MiddlewareConnector()

	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	dest_port_name = port_name + '/destination'

	ob['Init_OK'] = False

	try:
		# Get the dictionary for the component's state
		robot_state_dict = GameLogic.robotDict[parent]
		ob['Init_OK'] = True
	except AttributeError:
		print ("Component Dictionary not found!")
		print ("This component must be part of a scene")

	if ob['Init_OK']:
		print ('######## CONTROL INITIALIZATION ########')
		robot_state_dict['destination'] = 0
		robot_state_dict['tolerance'] = 1.5
		robot_state_dict['path'] = deque()
		robot_state_dict['wp_index'] = 0
		#print ("OPENING PORT '{0}'".format(dest_port_name))
		GameLogic.orsConnector.registerBufferedPortBottle([dest_port_name])
		print ('######## CONTROL INITIALIZED ########')

		# Initialize the area object, according to the initial parameters
		try:
			scene = GameLogic.getCurrentScene()
			target_ob = scene.objects[ob['TargetObject']]
			area_ob = scene.objects['OBWP_Area']
			initial_position = target_ob.position
			initial_position[2] = 0.01
			area_ob.position = initial_position
			area_ob.setParent(target_ob)
		except KeyError:
			print ("Controller WARNING: No area object in scene")


def move(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	dest_port_name = port_name + '/destination'

	destination = []

	# Get the dictionary for the robot's state
	robot_state_dict = GameLogic.robotDict[parent]

	if ob['Init_OK']:

		# Reset movement variables
		vx, vy, vz = 0.0, 0.0, 0.0
		rx, ry, rz = 0.0, 0.0, 0.0

	############################### SPEED #################################


		"""
		# Read the destination coordinates as a bottle
		# Retrieve the port we want to read from
		p = GameLogic.orsConnector.getPort(dest_port_name)

		# Non-blocking read of the port
		dest = p.read(False)

		if dest!=None:
			for i in range(3):
				destination.append( dest.get(i).asDouble() )
		"""

		# Read the path coordinates as a list of dictionaries
		# Define a list with the data we are expecting
		data_types = ['string']
		message = GameLogic.orsConnector.readMessage(data_types, dest_port_name)
		if message != None:
			# The 'readMessage' function returns a list,
			#  with as many items as specified in the 'data_types' list.
			# In this case the list has only one item
			json_data = message[0]

			# Decode the JSON string
			wp_list = json.loads (json_data, encoding='UTF-8')
			# Build a queue of the waypoints, called path
			for wp in wp_list:
				way_point = wp['point']
				way_point['tolerance'] = wp['radius']
				robot_state_dict['path'].append(way_point)

			# Extract the cordinates of the next waypoint
			scene = GameLogic.getCurrentScene()
			target_ob = scene.objects[ob['TargetObject']]
			destination, tolerance = get_next_waypoint(robot_state_dict['path'], robot_state_dict, target_ob)
			resize_area (tolerance)
			status = "Moving to {0}".format(robot_state_dict['wp_index'])
			change_status(contr, robot_state_dict, status, parent)
			print ("Controller: Got destination: " + repr(destination))
			robot_state_dict['destination'] = destination
			robot_state_dict['tolerance'] = tolerance


		try:
			# If the robot has received a command to stop
			#  then set its speed to 0
			if robot_state_dict['moveStatus'] == "Stop" and robot_state_dict['vx'] != 0:
				robot_state_dict['vx'] = vx
				robot_state_dict['rz'] = rz
				change_speed (contr, parent)

			# Exit the function if there has been no command to move
			if not robot_state_dict['moveStatus'][:9] == "Moving to":
				return
		except KeyError:
			# Also exit if there is no moveStatus property
			#print ("Controller WARNING: No porperty 'moveStatus' in the robot dictionary")
			return

		# Get the location information from the target and area objects
		scene = GameLogic.getCurrentScene()
		target_ob = scene.objects[ob['TargetObject']]
		destination = target_ob.position
		tolerance = robot_state_dict['tolerance']
		# Ignore the altitude (Z)
		destination[2] = 0

		# Calculate the direction needed
		location_V = Mathutils.Vector(ob.position)
		# Ignore the altitude (Z)
		location_V[2] = 0
		destination_V = Mathutils.Vector(destination)

		distance_V = destination_V - location_V
		distance = distance_V.length - tolerance

		#print ("GOT DISTANCE: %.4f" % distance)

		try:
			robot_angle = robot_state_dict['gyro_angle']
		except KeyError as detail:
			#print (detail)
			# Force the robot to stay put, since we can't orient it
			change_status(contr, robot_state_dict, "Stop", parent)
			print ("Controller WARNING: Robot orientation angle not found. The robot needs to have a Gyroscope to navigate. Aborting move.")
			return

		# Compute the angle of the robot wrt the target
		world_X_vector = Mathutils.Vector([1,0,0])
		world_Y_vector = Mathutils.Vector([0,1,0])
		distance_V.normalize()
		target_angle = Mathutils.AngleBetweenVecs(distance_V, world_X_vector)
		# Correct the direction of the turn according to the angles
		dot = distance_V.dot(world_Y_vector)
		if dot < 0:
			target_angle = target_angle * -1

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

		if ob.name == "OBATRV2":
			print ("ATRV2 TOLERANCE = {0}".format(tolerance))

		if distance > 0:
			# Move forward
			vx = 0.05
			# Test if the orientation of the robot is within tolerance
			if not (-angle_tolerance < angle_diff and angle_diff < angle_tolerance):
				# If not, rotate the robot
				rz = 0.03 * rotation_direction
		# If the target has been reached, change the status
		elif distance <= 0:
			if ob.name == "OBATRV2":
				print "ATRV2 ARRIVED AT DESTINATION"
			# Switch to the next waypoint
			scene = GameLogic.getCurrentScene()
			target_ob = scene.objects[ob['TargetObject']]
			destination, tolerance = get_next_waypoint(robot_state_dict['path'], robot_state_dict, target_ob)
			if destination == None:
				# If there are no more waypoints, the goal has been reached
				change_status(contr, robot_state_dict, "Arrived", parent)
				print ("Controller: Robot {0} reached its target".format(parent))
			else:
				print ("Controller: Robot {0} new destination: {1}".format(parent, repr(destination)))
				robot_state_dict['destination'] = destination
				robot_state_dict['tolerance'] = tolerance
				resize_area (1)
				resize_area (tolerance)
				status = "Moving to {0}".format(robot_state_dict['wp_index'])
				change_status(contr, robot_state_dict, status, parent)

		# Set the values in the robot dictionary
		robot_state_dict['vx'] = vx
		robot_state_dict['vy'] = vy
		robot_state_dict['vz'] = vz

		robot_state_dict['rx'] = rx
		robot_state_dict['ry'] = ry
		robot_state_dict['rz'] = rz

		change_speed (contr, parent)

		#print ("Motion for robot '{0}'".format(parent.name))
		#print ("\tvx: ",vx," vy: ",vy," vz: ",vz)
		#print ("\trx: ",rx," ry: ",ry," rz: ",rz)


def change_speed (contr, parent):
	""" Send the blender message to the robot
		to update its movement variables. """
	msg_act = contr.actuators['Send_update_msg']
	msg_act.subject = 'Speed'
	msg_act.body = parent.name
	contr.activate(msg_act)


def change_status (contr, robot_state_dict, new_status, parent):
	""" Update the robot's movement status in the dictionary.
		Also send a message to the parent indicating the change. """
	msg_act = contr.actuators['Send_status_msg']
	msg_act.subject = 'Status'
	msg_act.body = parent.name
	contr.activate(msg_act)

	robot_state_dict['moveStatus'] = new_status
	print ("Controller: Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus']))


def resize_area (tolerance):
	""" Scale the area object to the tolerance of the waypoint. """
	scene = GameLogic.getCurrentScene()
	try:
		area_ob = scene.objects['OBWP_Area']
		area_ob.scaling = (tolerance, tolerance, 1)
	except KeyError:
		#print ("Controller WARNING: No area object in scene")
		pass


def get_next_waypoint (path, robot_state_dict, target_ob):
	""" Pop the next waypoint from the queue, and fill the variables. """

	destination = []
	try:
		way_point = path.popleft()
		destination.append( float(way_point['x']) )
		destination.append( float(way_point['y']) )
		destination.append( float(way_point['z']) )
		tolerance = float(way_point['tolerance'])

		# Translate the marker to the target destination
		destination[2] = 0
		target_ob.position = destination
		robot_state_dict['wp_index'] = robot_state_dict['wp_index'] + 1

		return destination, tolerance
	except IndexError:
		return None, None

