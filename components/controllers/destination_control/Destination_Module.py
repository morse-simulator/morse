import sys, os
import GameLogic
import Mathutils

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

# Radius of tolerance for waypoints
# Given in Blender units
tolerance = 2

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
		print "Component Dictionary not found!"
		print "This component must be part of a scene"

	if ob['Init_OK']:
		print '######## CONTROL INITIALIZATION ########'
		print "OPENING PORT '{0}'".format(dest_port_name)
		GameLogic.orsConnector.registerBufferedPortBottle([dest_port_name])
		print '######## CONTROL INITIALIZED ########'

		# Initialize the area object, according to the initial parameters
		try:
			scene = GameLogic.getCurrentScene()
			target_ob = scene.objects[ob['TargetObject']]
			area_ob = scene.objects['OBWP_Area']
			area_ob.scaling = (tolerance, tolerance, 1)
			initial_position = target_ob.position
			initial_position[2] = 0.01
			area_ob.position = initial_position
			area_ob.setParent(target_ob)
		except KeyError:
			print "Warning: No area object in scene"



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
		#retrieve the port we want to write on
		p = GameLogic.orsConnector.getPort(dest_port_name)

		#non-blocking read of the port
		dest = p.read(False)

		if dest!=None:
			for i in range(3):
				destination.append( dest.get(i).asDouble() )

			robot_state_dict['moveStatus'] = "Transit"
			print "GOT DESTINATION: ", destination
			print "Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus'])
			robot_state_dict['destination'] = destination

			# DEBUGGING:
			# Translate the marker to the target destination
			destination[2] = 0
			scene = GameLogic.getCurrentScene()
			target_ob = scene.objects[ob['TargetObject']]
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
		distance = distance_V.length - tolerance

		#print "GOT DISTANCE: ", distance

		world_X_vector = Mathutils.Vector([1,0,0])
		world_Y_vector = Mathutils.Vector([0,1,0])
		distance_V.normalize()
		target_angle = Mathutils.AngleBetweenVecs(distance_V, world_X_vector)
		# Correct the direction of the turn according to the angles
		dot = distance_V.dot(world_Y_vector)
		if dot < 0:
			target_angle = target_angle * -1

		try:
			robot_angle = robot_state_dict['gyro_angle']
		except KeyError as detail:
			print "Gyroscope angle not found. Does the robot have a Gyroscope?"
			print detail
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

		#print "Angles: R=%.4f, T=%.4f  Diff=%.4f  Direction = %d" % (robot_angle, target_angle, angle_diff, rotation_direction)

		if distance > 0:
			# Move forward
			vx = 0.05
			# Test if the orientation of the robot is within tolerance
			# If not, rotate the robot
			if not (-angle_tolerance < angle_diff and angle_diff < angle_tolerance):
				rz = 0.03 * rotation_direction
		# If the target has been reached, change the status
		elif distance <= 0:
			robot_state_dict['moveStatus'] = "Stop"
			print "TARGET REACHED"
			print "Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus'])

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

		#print "Motion for robot '{0}'".format(parent.name)
		#print "\tvx: ",vx," vy: ",vy," vz: ",vz
		#print "\trx: ",rx," ry: ",ry," rz: ",rz
