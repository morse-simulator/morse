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
			change_status (contr, robot_state_dict, "Teleported", parent)



def change_status (contr, robot_state_dict, new_status, parent):
	""" Update the robot's movement status in the dictionary.
		Also send a message to the parent indicating the change. """
	msg_act = contr.actuators['Send_status_msg']
	msg_act.subject = 'Status'
	msg_act.body = parent.name
	contr.activate(msg_act)

	robot_state_dict['moveStatus'] = new_status
	print ("Controller: Robot {0} move status: '{1}'".format(parent, robot_state_dict['moveStatus']))


