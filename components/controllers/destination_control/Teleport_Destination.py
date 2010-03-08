import sys, os
import GameLogic
import Mathutils
from middleware.independent.IndependentBlender import *
import setup.ObjectData


def move(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	dest_port_name = port_name + '/in'

	# Radius of tolerance for waypoints
	tolerance = 2
	destination = []

	# Direction tolerance for the movement (in degrees)
	angle_tolerance = 5

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
			print "TELEPORT GOT DESTINATION: ", destination

			# DEBUGGING:
			# Translate the marker to the target destination
			destination[2] = 0
			parent.position = destination
