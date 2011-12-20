import sys, os
import json
import re
import yarp

try:
   scriptRoot = os.path.join(os.environ['ORS_ROOT'],'scripts')
except KeyError:
   scriptRoot = '.'

try:
   libRoot = os.path.join(os.environ['ORS_ROOT'],'lib')
except KeyError:
   libRoot = '.'

sys.path.append(scriptRoot)
sys.path.append(libRoot)

from middleware.independent.IndependentBlender import *

# Global variables for the names of the input and output ports
ors_in_port_name = "/ors/admin/in"
#ors_out_port_name = "/ors/robots/OBATRV/Gyroscope"
ors_out_port_name = "/ors/admin/out"

local_in_port_name = "/scene/admin/in"
local_out_port_name = "/scene/admin/out"

orsConnector = MiddlewareConnector()

local_in_port = 0
local_out_port = 0


def get_robot_data():
	bottle = local_out_port.prepare()
	bottle.clear()
	bottle.addString("list_robots")
	local_out_port.write(False)

	waiting = True

	print ("Sent the request")

	while waiting:
		robot_data = local_in_port.read(False)
		if robot_data != None:

			"""
			# Reading an integer from the bottle
			gyro = robot_data.get(0).asInt()
			print ("Decripted data: '{0}'".format(gyro))
			"""

			# Reading a string from the bottle
			json_data = robot_data.toString()
			print ("Encripted data: '{0}'".format(json_data))
			# Remove the quotations at the start and end of the string
			json_data = re.sub(r'"(.*)"', r'\1', json_data)
			# Unescape all other quotation marks
			json_data = re.sub(r'\\(.)', r'\1', json_data)
			print ("Unescaped data: '{0}'".format(json_data))
			robot_list = json.loads(json_data)
			print ("Decripted data: '{0}'".format(robot_list))


			for item in robot_list:
				name, components = item
				print ("\tRobot: {0}".format(name))
				for comp in components:
					print ("\t\t{0}".format(comp))

			waiting = False


def port_setup():
	global local_in_port
	global local_out_port

	yarp.Network.init()

	local_in_port = yarp.BufferedPortBottle()
	local_in_port.open(local_in_port_name)
	local_out_port = yarp.BufferedPortBottle()
	local_out_port.open(local_out_port_name)

	yarp.Network.connect (local_out_port_name, ors_in_port_name)
	yarp.Network.connect (ors_out_port_name, local_in_port_name)


port_setup()
get_robot_data()
