import sys, os
import json
import re
import yarp


# Global variables for the names of the input and output ports
local_in_port_name = ""
local_out_port_name = ""

# Global variables for the ors names of the ports
ors_in_port_name = ""
ors_out_port_name = ""

# Global variables for the yarp ports
local_in_port = 0
local_out_port = 0



def read_trajectory():
	""" Read a list of waypoints from a file.
		Format the data as a list of dictionaries,
		in the same format as the java objects from ROSACE."""
	filename = "qarm.traj"
	file = open(filename, "r")
	arm_movement = []

	for line in file:
		# Get the individual elements, splitting by whitespace
		rotation = line.split()

		arm_movement.append (rotation)

	return arm_movement


def command_robot():
	waiting = True

	# Get the contents of the trajectory config file
	arm_movement = read_trajectory()
	i = 0

	"""
	arm_movement = [	[1, 0, 0, 0, 0, 0],
						[0, 1, 0, 0, 0, 0],
						[0, 0, 1, 0, 0, 0],
						[0, 0, 0, 1, 0, 0],
						[0, 0, 0, 0, 1, 0],
						[0, 0, 0, 0, 0, 1],
						[0, 0, 0, 0, 0, 0],
					]
	"""

	print ("SEQUENCE READ: {0}".format(arm_movement))
	while waiting:
		#raw_coords = input("Input new coordinates: ")
		#print ("The coordinates read are: {0}, of type ({1})".format(raw_coords, type(raw_coords)))

		command = raw_input("Enter command: [ (s)tep / e(x)it ] ")

		if command == "move" or command == "m":
			command = {"command":"move"}
			send_command(command)

		elif command == "step" or command == "s":
			movement = arm_movement[i]
			print ("SENDING MOVEMENT: {0}".format(movement))
			i = i + 1
			send_movement(movement)

		elif command == "exit" or command == "x":
			print ("Exiting the function")
			sys.exit()

def send_movement(movement):
	""" Send the rotation array to the arm """
	bottle = local_out_port.prepare()
	bottle.clear()
	for item in movement:
		#bottle.addDouble(item)
		bottle.addDouble(eval(item))
	local_out_port.write(False)


def send_command(command):
	""" Send a message through the yarp output port."""
	bottle = local_out_port.prepare()
	bottle.clear()
	bottle.addString(command)
	local_out_port.write(False)


def port_setup(robot_name):
	""" Open the input and output ports."""
	global local_in_port
	global local_out_port

	global local_in_port_name
	global local_out_port_name

	global ors_in_port_name
	global ors_out_port_name

	# Define the names for all the ports
	port_prefix = "/ors/robots/" + robot_name + "/"
	local_port_prefix = "/pa10_client/" + robot_name + "/"

	ors_in_port_name = port_prefix + "OBPA-10/in"
	ors_out_port_name = port_prefix + "out"

	local_in_port_name = local_port_prefix + "in/"
	local_out_port_name = local_port_prefix + "out/"

	# Start the yarp network connection
	yarp.Network.init()

	# Open the client ports
	local_in_port = yarp.BufferedPortBottle()
	local_in_port.open(local_in_port_name)
	local_out_port = yarp.BufferedPortBottle()
	local_out_port.open(local_out_port_name)

	# Connect the client ports to the simulator ports
	yarp.Network.connect (local_out_port_name, ors_in_port_name)
	yarp.Network.connect (ors_out_port_name, local_in_port_name)



def usage(program_name):
	print ("Usage: {0} [robot_name]\n", program_name)


def main():
	print ("********* PA-10 arm client *********")

	robot_name = "OBJido"

	argc = len(sys.argv)

	if argc == 2:
		robot_name = sys.argv[1]
	elif argc > 2:
		usage(sys.argv[0])
		sys.exit()

	port_setup(robot_name)

	print (" * Writing commands to " + ors_in_port_name)
	print (" * Listening status on " + ors_out_port_name)

	print (" * Enter command:")

	command_robot()


if __name__ == "__main__":
    main()
