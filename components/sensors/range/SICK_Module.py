import sys, os
import GameLogic
import Mathutils
import json
import math


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


def init(contr):
	# Middleware initialization
	if not hasattr(GameLogic, 'orsConnector'):
		GameLogic.orsConnector = MiddlewareConnector()

	# Get the object data
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)

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
		GameLogic.orsConnector.registerBufferedPortBottle([port_name])
		print ('######## SICK INITIALIZED ########')


def laser_sweep(contr):
	""" Do ray tracing from the SICK object towards the Ray_arrow object.
		Return the points located. """
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)

	if ob['Init_OK']:
		robot_state_dict = GameLogic.robotDict[parent]

		############### SICK laser ###################
		scene = GameLogic.getCurrentScene()
		ray_ob = scene.objects['OBRay_arrow']
		target,point,normal = ob.rayCast(ray_ob,None,ob['Laser_Range'])

		if target:
			#print ("Laser detected: Object {0}, direction {1}".format(target, point))

			# Define the message structure to send.
			# It is a list of tuples (data, type).
			sick_dict = {'point': point}
			message = json.dumps(sick_dict)
			message_data = [ (message, 'string') ]

			GameLogic.orsConnector.postMessage(message_data, port_name)

			# Add a point to the location of the ray intersection
			ray_int_ob = scene.objects['OBRay_target']
			ray_int_ob.position = point
			intersect_act = contr.actuators['Register_intersection']
			contr.activate(intersect_act)


def arc_sweep(contr):
	""" Do ray tracing from the SICK object using a semicircle
		Return the points located. """
	ob, parent, port_name = setup.ObjectData.get_object_data(contr)

	if ob['Init_OK']:
		robot_state_dict = GameLogic.robotDict[parent]

		############### SICK laser ###################
		scene = GameLogic.getCurrentScene()

		# Retrieve the arc object to use as the base of the rays
		ray_arc = robot_state_dict['sick_arc']
		# Set its visibility, according to the settings
		ray_arc.setVisible(ob['Visible_arc'])

		# Obtain the rotation matrix of the sensor.
		rot_matrix = ob.worldOrientation
		inverted_matrix = Mathutils.Matrix(rot_matrix[0], rot_matrix[1], rot_matrix[2])
		# According to the GE documentation, it has to be inverted first
		inverted_matrix.transpose()
		# Then invert it, to use it to find the arc vertices
		inverted_matrix.invert()

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
						point_list.append(point_string)

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

		# Write the detected points to a file
		filename = "{0}_{1}.txt".format(parent,ob)
		parent_position = "[%.4f, %.4f, %.4f]\n" % (parent.position[0], parent.position[1], parent.position[2])
		write_points_to_file(filename, point_list, parent_position)


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
