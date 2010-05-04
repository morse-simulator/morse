import sys, os
import GameLogic
import Mathutils
import json
import math

import time
from Sick_Poster import ors_sick_poster
from datetime import datetime;
from helpers.MorseTransformation import Transformation3d


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
from helpers import MorseMath


def init(contr):
	# Middleware initialization
	if not hasattr(GameLogic, 'orsConnector'):
		GameLogic.orsConnector = MiddlewareConnector()

	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	port_name = port_name + "/out"

	ob['Init_OK'] = False

	try:
		# Get the dictionary for the component's state
		robot_state_dict = GameLogic.robotDict[parent]
		# Look for a child arc to use for the scans
		for child in ob.children:
			if child.name[:6] == 'OBArc_':
				robot_state_dict['sick_arc'] = child
				ob['Init_OK'] = True
				break
			elif child.name == 'OBRay':
				ob['Init_OK'] = True
	except AttributeError:
		print ("Component Dictionary not found!")
		print ("This component must be part of a scene")

	if ob['Init_OK']:
		print ('######## SICK INITIALIZATION ########')
		print ("Sick: Using arc object: '{0}'".format(robot_state_dict['sick_arc']))
		# Prepare a list with the data for the header of the file
		data = []
		data.append("SICK on robot {0}\n".format(parent))
		data.append("(distance, globalVector(3), localVector(3))\n")
		data.append(repr(ob.getVectTo(parent)) + "\n")

		# Open the output file, only to erase its previous contents
		filename = "{0}_{1}.txt".format(parent,ob)
		print ("Sick: Writing to file: '{0}'".format(filename))
		FILE = open(filename, 'wb')
		FILE.writelines(data)
		FILE.close()

		# YARP port
		GameLogic.orsConnector.registerBufferedPortBottle([port_name])

		### POCOLIBS ###
		poster_name = "sickMorse"
		robot_state_dict[port_name] = ors_sick_poster.init_data(poster_name)
		print ("Poster ID generated: {0}".format(robot_state_dict[port_name]))
		if robot_state_dict[port_name] == None:
			print ("ERROR creating poster. This module may not work")
			ob['Init_OK'] = False

		print ('######## SICK INITIALIZED ########')


def arc_sweep(contr):
	""" Do ray tracing from the SICK object using a semicircle
		Return the points located. """
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)
	port_name = port_name + "/out"

	if ob['Init_OK']:
		robot_state_dict = GameLogic.robotDict[parent]

		try:
			#### ONLY FOR THE GRID TESTS ####
			sensor = contr.sensors['Trigger']
			# Exit the function unless the signal is positive
			if not sensor.positive:
				return
		except KeyError:
			pass
			


		############### SICK laser ###################
		scene = GameLogic.getCurrentScene()

		# Retrieve the arc object to use as the base of the rays
		ray_arc = robot_state_dict['sick_arc']
		# Set its visibility, according to the settings
		ray_arc.setVisible(ob['Visible_arc'])

		# Obtain the rotation matrix of the sensor.
		inverted_matrix = MorseMath.invert_rotation_matrix(ob)

		# Create a vector for the Mathutils operations
		vector_point = Mathutils.Vector()

		# Create an empty list to store the intersection points
		point_list = []

		# Get the mesh for the semicircle
		for mesh in ray_arc.meshes:
			for mat in range(mesh.numMaterials):
				for v_index in range(mesh.getVertexArrayLength(mat)):
					vertex = mesh.getVertex(mat, v_index)
					vertex_pos = vertex.getXYZ()

					#print ("\tORIGINAL POINT: [%.2f, %.2f, %.2f]" % (vertex_pos[0], vertex_pos[1], vertex_pos[2]))

					# Skip the center vertex
					# NOTE: Make sure the center vertex of the arc
					#  has local coordinates 0.0, 0.0, 0.0
					if vertex_pos == [0.0, 0.0, 0.0]:
						continue

					# Adjust the vector coordinates to the rotation
					#  of the robot
					fill_vector (vector_point, vertex_pos)
					corrected_vertex = ob.getAxisVect(vector_point)

					ray = ob.position
					# Displace according to the arc vertices
					for i in range(3):
						ray[i] = ray[i] + corrected_vertex[i]

					#print ("\tv: [%.2f, %.2f, %.2f]\t\tr: [%.2f, %.2f, %.2f]" % (vertex_pos[0], vertex_pos[1], vertex_pos[2], ray[0], ray[1], ray[2]))

					# Shoot a ray towards the target
					target,point,normal = ob.rayCast(ray,None,ob['Laser_Range'])

					# If there was an intersection,
					#  send the vertex to that point
					if target:
						#print ("\tGOT INTERSECTION WITH RAY: [%.2f, %.2f, %.2f]" % (ray[0], ray[1], ray[2]))
						#print ("\tINTERSECTION AT: [%.2f, %.2f, %.2f] = %s" % (point[0], point[1], point[2], target))

						# Substract the sensor coordinates
						#  from the intersection point
						for i in range(3):
							point[i] = point[i] - ob.position[i]
						#print ("\tARC POINT: [%.2f, %.2f, %.2f]" % (point[0], point[1], point[2]))

						# Create a vector object
						fill_vector (vector_point, point)

						# Multiply the resulting point by the inverse
						#  of the sensor rotation matrix
						arc_point = vector_point * inverted_matrix
						#print ("\tARC POINT: [%.2f, %.2f, %.2f]" % (arc_point[0], arc_point[1], arc_point[2]))

						point_string = "[%.4f, %.4f, %.4f]\n" % (arc_point[0], arc_point[1], arc_point[2])

						# Do not move the point if the ray intersection
						#  happened at the origin
						#  (because this breaks the arc and makes all
						#  subsequent rays wrong)
						if valid_range (arc_point, 0.1):
							# Send the vertex to the new location
							vertex.setXYZ(arc_point)

					# Otherwise return the vertex to its original position
					else:
						# Create a vector object
						fill_vector (vector_point, vertex_pos)
						# Give it the correct size
						vector_point.normalize()
						vector_point = vector_point * ob['Laser_Range']

						#print ("\tVECTOR POINT: [%.2f, %.2f, %.2f]" % (vector_point[0], vector_point[1], vector_point[2]))
						# Move the vertex to the computed position
						vertex.setXYZ(vector_point)

						# Add a point at 0,0,0 to the output file,
						#  to mark that this ray did not find anything
						point_string = "[%.4f, %.4f, %.4f]\n" % (0.0, 0.0, 0.0)

					point_list.append(point_string)

		# Write the detected points to a file
		filename = "{0}_{1}.txt".format(parent,ob)
		try:
			parent_position = "[%.4f, %.4f, %.4f], %.4f\n" % (parent.position[0], parent.position[1], parent.position[2], robot_state_dict['Yaw'])
			write_points_to_file(filename, point_list, parent_position)
		except KeyError:
			print "SICK: Gyroscope not available"


		### POCOLIBS ###
		mainToOrigin = Transformation3d(parent)
		sensorToOrigin = Transformation3d(ob)
		mainToSensor = mainToOrigin.transformation3dWith(sensorToOrigin)

		pom_robot_position =  ors_sick_poster.pom_position()
		pom_robot_position.x = mainToOrigin.x
		pom_robot_position.y = mainToOrigin.y
		pom_robot_position.z = mainToOrigin.z
		pom_robot_position.yaw = robot_state_dict['Yaw']
		pom_robot_position.pitch = robot_state_dict['Pitch']
		pom_robot_position.roll = robot_state_dict['Roll']

		pom_sensor_position = ors_sick_poster.pom_position()
		pom_sensor_position.x = mainToSensor.x
		pom_sensor_position.y = mainToSensor.y
		pom_sensor_position.z = mainToSensor.z
		# XXX +PI rotation is needed but I don't have any idea why !!
		pom_sensor_position.yaw = mainToSensor.yaw + 180.0
		pom_sensor_position.pitch = mainToSensor.pitch
		pom_sensor_position.roll = mainToSensor.roll

		# Compute the current time ( we only requiere that the pom date
		# increases using a constant step so real time is ok)
		t = datetime.now()
		pom_date = int(t.hour * 3600* 1000 + t.minute * 60 * 1000 +
				t.second * 1000 + t.microsecond / 1000)

		pom_pos_data = ors_sick_poster.POM_SENSOR_POS()
		ors_sick_poster.create_pom_sensor_pos(pom_date, pom_pos_data, pom_robot_position, pom_sensor_position)


		# Fill in the structure with the image information
		sensor_info = ors_sick_poster.SICK_MEASURES_HEADER_STR()
		#sensor_info = ors_sick_poster.sick_struct()
		sensor_info.np = len(point_list)
		# Frame is one of
		#  ( SICK_SICK_FRAME, SICK_ROBOT_FRAME, SICK_GLOBAL_FRAME )
		sensor_info.frame = ors_sick_poster.SICK_SICK_FRAME
		sensor_info.start_time = t.second
		sensor_info.rcv_time = t.second
		sensor_info.theta_min = 0.0
		sensor_info.dtheta = math.radians(1) # Convert 1 degree to radians
		sensor_info.sickPomPos = pom_pos_data

		# Create the poster with the data for both images
		posted = ors_sick_poster.post_sick_poster(robot_state_dict[port_name], sensor_info, sensor_data)




def valid_range(point_vector, radius):
	""" A ray intersection will only be valid if it happens
		outside of a certain radius from the source.
		This radius should be equivalent to the size of
		the laser emiter. """
	if point_vector.length < radius:
		return False
	else:
		return True


def fill_vector(vector, point_list):
	""" Copy the contents of a list into an existing vector structure. """
	for i in range(3):
		vector[i] = point_list[i]


def write_points_to_file(filename, point_list, robot_position):
	""" Write the contents of the list to a file.
		This function will be called once for every laser sweep. """
	FILE = open(filename, 'ab')
	FILE.write("==> SCAN AT LOCATION: {0}".format(robot_position))
	FILE.writelines(point_list)
	FILE.close()
