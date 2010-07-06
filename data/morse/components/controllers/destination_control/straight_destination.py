import sys, os
import GameLogic
import Mathutils
from middleware.independent.IndependentBlender import *
import setup.ObjectData
from helpers import MorseMath


def move(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	dest_port_name = port_name + '/in'
	speed_port_name = port_name + '/speed/in'

	destination = []

	# Direction tolerance for the movement (in degrees)
	angle_tolerance = 5

	# Get the dictionary for the robot's state
	robot_state_dict = GameLogic.robotDict[parent]


	# Speed variable
	speed = robot_state_dict['speed']

	NED = False
	# Get the orientation of the robot
	if parent['Orientation'] == 'NED':
		NED = True

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
			print ("SETTING SPEED TO: {0}".format(speed))


	    ########################### DESTINATION ###############################
		# Retrieve the port we want to read from
		p = GameLogic.orsConnector.getPort(dest_port_name)

		#non-blocking read of the port
		dest = p.read(False)

		if dest!=None:
			for i in range(3):
				destination.append( dest.get(i).asDouble() )

			robot_state_dict['moveStatus'] = "Transit"
			print ("STRAIGHT GOT DESTINATION: {0}".format(destination))
			print ("Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus']))
			robot_state_dict['destination'] = destination

			# DEBUGGING:
			# Translate the marker to the target destination
			scene = GameLogic.getCurrentScene()
			target_ob = scene.objects[ob['TargetObject']]
			if NED == True:
				d=destination[0]
				destination[0] = destination[1]
				destination[1] = d				
				destination[2] = -destination[2]

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
		#destination[2] = destination[2]

		# Ignore the altitude (Z)
		#destination[2] = 0

		# Calculate the direction needed
		location_V = Mathutils.Vector(ob.position)
		
		# Ignore the altitude (Z)
		#location_V[2] = 0
		destination_V = Mathutils.Vector(destination)				

		distance_V = destination_V - location_V
		#print ("\nlocation_V {0}".format(location_V))
		#print ("destination_V {0}".format(destination_V))
		#print ("distance_V {0}".format(distance_V))
		distance = distance_V.length - robot_state_dict['tolerance']

		#print ("GOT DISTANCE: {0}".format(distance))

		# Testing to get the correct transformation for the
		#  movement of the robot using its local coordinate system
		# The results were strange either using local or global coordinates
		#rotation_matrix = MorseMath.get_rotation_matrix (parent)
		#MorseMath.print_matrix_33(rotation_matrix)
		#distance_V = distance_V * rotation_matrix
		#print ("ROTATION distance_V {0}".format(distance_V))
		#inverted_matrix = MorseMath.invert_rotation_matrix (parent)
		#MorseMath.print_matrix_33(inverted_matrix)
		#distance_V = inverted_matrix * distance_V
		#print ("INVERTED distance_V {0}".format(distance_V))


		if distance > 0:
			# Move forward
			distance_V.normalize()
			#fps = GameLogic.getAverageFrameRate()
			ticks = GameLogic.getLogicTicRate()
			
			if NED == True:
				vx = distance_V[1] * speed/ticks
				vy = distance_V[0] * speed/ticks
				vz = -distance_V[2] * speed/ticks
			else:
				vx = distance_V[0] * speed/ticks
				vy = distance_V[1] * speed/ticks
				vz = distance_V[2] * speed/ticks

			# Correction of the movement direction,
			#  with respect to the object's orientation
			#movement_V = Mathutils.Vector(vx, vy, vz)
			#(vx, vy, vz) = parent.getAxisVect(movement_V)

				
		# If the target has been reached, change the status
		elif distance <= 0:
			# Teleport robot to the desired destination
			parent.position = target_ob.position

			robot_state_dict['moveStatus'] = "Stop"
			print ("TARGET REACHED")
			print ("Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus']))

		"""
		robot_state_dict['vx'] = vx
		robot_state_dict['vy'] = vy
		robot_state_dict['vz'] = vz

		msg_act = contr.actuators['Send_update_msg']
		msg_act.propName = parent.name
		msg_act.subject = 'Speed'
		contr.activate(msg_act)
		"""

		# Give the movement instructions directly to the parent
		# The second parameter specifies a "global" movement
		parent.applyMovement([vx, vy, vz], False)

		#print ("Motion for robot '{0}'".format(parent.name))
		#print ("\tvx: %.4f, %4f, %4f" % (vx, vy, vz))
		#print ("\trx: %.4f, %4f, %4f" % (rx, ry, rz))
