import sys, os
import GameLogic
import Mathutils
from middleware.independent.IndependentBlender import *
import setup.ObjectData


def move(contr):
	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	dest_port_name = port_name + '/in'

	destination = []

	# Get the dictionary for the robot's state
	robot_state_dict = GameLogic.robotDict[parent]

	if ob['Init_OK']:

	    ######## DESTINATION #########
		# Retrieve the port we want to read from
		p = GameLogic.orsConnector.getPort(dest_port_name)

		# Non-blocking read of the port
		dest = p.read(False)

		if dest!=None:
			for i in range(3):
				destination.append( dest.get(i).asDouble() )

			print "TELEPORT GOT DESTINATION: ", destination
			#destination[2] = 0
			parent.position = destination
