import math
import GameLogic
from morse.middleware.pocolibs.sensors.Viman_Poster import ors_viman_poster

def init_extra_module(self, component_instance, function):
	""" Setup the middleware connection with this data

	Prepare the middleware to handle the serialised data as necessary.
	"""
	# Compose the name of the poster, based on the parent and module names
	component_name = component_instance.blender_obj.name
	parent_name = component_instance.robot_parent.blender_obj.name
	poster_name = 'viman_{0}_{1}_Poster'.format(parent_name, component_name)
	poster_id = init_viman_poster(self, component_instance, poster_name)

	if poster_id != None:
		print ("Pocolibs created poster '%s' of type viman" % poster_id)
		component_instance.output_functions.append(function)
		# Store the name of the port
		self._poster_dict[component_name] = poster_id


def init_viman_poster(self, component_instance, poster_name):
	""" Prepare the data for a viman poster """

	# Create the predefined list of objects:
	self.scene_object_list = ["TRASHBIN", "SHELF", "LOWTABLE", "CUPHANDLE", "ACCESSKIT", "HRP2TABLE", "OBJ_CALIB", "SIMPLE_OBJECT", "SPACENAVBOX", "YELLOW_BOTTLE", "ORANGE_BOTTLE", "BLUE_BOTTLE", "GRIPPER", "ORANGEBOX", "WOODEN_OBJECT", "PINK_TRASHBIN", "GREY_TAPE", "BLACK_TAPE", "GREY_K7"]

	self.viman_data = ors_viman_poster.create_viman_struct(self.scene_object_list, len(self.scene_object_list))


	"""
	# Init the data structures used by this poster
	viman_data = ors_viman_poster.VimanObjectArray()
	viman_data.nObjects = len(self.scene_object_list)
	i = 0
	for object in self.scene_object_list:
		viman_data.objects[i] = ors_viman_poster.VimanObject()
		viman_data.objects[i].name = object
		i = i + 1
	"""
	
	poster_id, ok = ors_viman_poster.init_data(poster_name)
	if ok == 0:
		print ("ERROR creating poster. This module may not work")
		return None

	print ("viman poster ID: {0}".format(poster_id))
	return poster_id


def write_viman(self, component_instance):
	""" Write an image and all its data to a poster """
	# Get the id of the poster already created
	poster_id = self._poster_dict[component_instance.blender_obj.name]
	parent = component_instance.robot_parent

	i = 0
	for object in self.scene_object_list:
		i = i + 1

		#self._create_world_matrix(object.worldOrientation, object.position, viman_data.objects[i].thetaMatOrigin)
		_fill_world_matrix(object.worldOrientation, object.position, i)

		#robot_matrix = viman_data.objects[i].thetaMatRobot
		#world_matrix = viman_data.objects[i].thetaMatOrigin
		#camera_matrix = viman_data.objects[i].thetaMatCam

	# Compute the current time
	#pom_date, t = self._compute_date()

	# Write to the poster with the data for all objects
	posted = ors_viman_poster.post_viman_poster(poster_id, self.viman_data)


def _fill_world_matrix(object_orientation_matrix, object_position, index):
	""" Fill the world matix part of the structure

	This function receives the Blender rotation matrix and position of an object
	It calls a module function to fill out data structure of type VimanThetaMat
	"""

	result = ors_viman_poster.write_matrix (self.viman_data, index,
		nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz)

	"""
	# THIS IS PROBABLY USELESS NOW
	viman_matrix.nx = object_orientation_matrix[0][0]
	viman_matrix.ox = object_orientation_matrix[0][1]
	viman_matrix.ax = object_orientation_matrix[0][2]
	viman_matrix.px = object_position[0]

	viman_matrix.ny = object_orientation_matrix[1][0]
	viman_matrix.oy = object_orientation_matrix[1][1]
	viman_matrix.ay = object_orientation_matrix[1][2]
	viman_matrix.py = object_position[1]

	viman_matrix.nz = object_orientation_matrix[2][0]
	viman_matrix.oz = object_orientation_matrix[2][1]
	viman_matrix.az = object_orientation_matrix[2][2]
	viman_matrix.pz = object_position[2]
	"""
