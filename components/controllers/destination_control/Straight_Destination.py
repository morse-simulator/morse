import sys, os
import GameLogic
import Mathutils
from middleware.independent.IndependentBlender import *
import setup.ObjectData


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
			print "STRAIGHT GOT DESTINATION: ", destination
			print "Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus'])
			robot_state_dict['destination'] = destination

			# DEBUGGING:
			# Translate the marker to the target destination
			scene = GameLogic.getCurrentScene()
			target_ob = scene.objects['OBWayPoint']
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
		target_ob = scene.objects['OBWayPoint']
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

		#print "GOT DISTANCE: ", distance




		if distance > 0:
			# Move forward
			distance_V.normalize()
			fps = GameLogic.getAverageFrameRate()
			if NED == True:
				vy = distance_V[0] * speed/fps
				vx = distance_V[1] * speed/fps
				vz = -distance_V[2] * speed/fps
			else:
				vx = distance_V[0] * speed/fps
				vy = distance_V[1] * speed/fps
				vz = distance_V[2] * speed/fps
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

		contr.activate(msg_act)

		#print "Motion for robot '{0}'".format(parent.name)
		#print "\tvx: ",vx," vy: ",vy," vz: ",vz
		#print "\trx: ",rx," ry: ",ry," rz: ",rz
