import math
import re
import GameLogic
from morse.middleware.pocolibs.sensors.Viman_Poster import ors_viman_poster

object_config_file = "objectList_cfg"

def init_extra_module(self, component_instance, function):
	""" Setup the middleware connection with this data

	Prepare the middleware to handle the serialised data as necessary.
	"""
	# Compose the name of the poster, based on the parent and module names
	component_name = component_instance.blender_obj.name
	parent_name = component_instance.robot_parent.blender_obj.name
	poster_name = 'viman_{0}_{1}'.format(parent_name, component_name)
	poster_id = init_viman_poster(self, component_instance, poster_name)

	if poster_id != None:
		#print ("Pocolibs created poster '%s' of type viman" % poster_id)
		component_instance.output_functions.append(function)
		# Store the name of the port
		self._poster_dict[component_name] = poster_id


def init_viman_poster(self, component_instance, poster_name):
	""" Prepare the data for a viman poster """

	self.scene_object_list = _read_object_list()

	if self.scene_object_list == None:
		print ("ERROR: VIMAN poster not configured properly. Disabling poster")
		return None

	self.viman_data = ors_viman_poster.generate_viman_struct()

	# Init the data structures used by this poster
	self.viman_data.nbObjects = len(self.scene_object_list)
	i = 0
	for object in self.scene_object_list:
		ors_viman_poster.set_name(self.viman_data, i, str(object))
		#self.viman_data.objects[i].name = object
		i = i + 1

	poster_id, ok = ors_viman_poster.init_data(poster_name)
	if ok == 0:
		print ("ERROR creating poster. This module may not work")
		return None

	#else:
	print("VIMAN Poster '%s' created (ID: %d)" % (poster_name, poster_id))

	return poster_id


def write_viman(self, component_instance):
	""" Write an image and all its data to a poster """
	# Get the id of the poster already created
	poster_id = self._poster_dict[component_instance.blender_obj.name]
	parent = component_instance.robot_parent

	scene = GameLogic.getCurrentScene()

	i = 0
	for object_id in self.scene_object_list:
		# Adjust the name of the components, while using Blender 2.49
		if GameLogic.pythonVersion < 3:
			object_id = 'OB' + object_id

		try:
			object = scene.objects[object_id]

			if object in component_instance.local_data['visible_objects']:
				ors_viman_poster.set_visible (self.viman_data, i, 1)
				_fill_world_matrix(self.viman_data, object.worldOrientation, object.position, i)
			else:
				ors_viman_poster.set_visible (self.viman_data, i, 0)

			#robot_matrix = viman_data.objects[i].thetaMatRobot
			#world_matrix = viman_data.objects[i].thetaMatOrigin
			#camera_matrix = viman_data.objects[i].thetaMatCam

			# Compute the current time
			#pom_date, t = self._compute_date()

			# Write to the poster with the data for all objects
			posted = ors_viman_poster.real_post_viman_poster(poster_id, self.viman_data)
		except KeyError as detail:
			#print ("WARNING: Object %s not found in the scene" % detail)
			pass
			posted = False

		i = i + 1


def _fill_world_matrix(viman_data, object_orientation_matrix, object_position, index):
	""" Fill the world matix part of the structure

	This function receives the Blender rotation matrix and position of an object
	It calls a module function to fill out data structure of type VimanThetaMat
	"""

	nx = object_orientation_matrix[0][0]
	ox = object_orientation_matrix[0][1]
	ax = object_orientation_matrix[0][2]
	px = object_position[0]

	ny = object_orientation_matrix[1][0]
	oy = object_orientation_matrix[1][1]
	ay = object_orientation_matrix[1][2]
	py = object_position[1]

	nz = object_orientation_matrix[2][0]
	oz = object_orientation_matrix[2][1]
	az = object_orientation_matrix[2][2]
	pz = object_position[2]

	result = ors_viman_poster.write_matrix (viman_data, index,
		nx, ny, nz, ox, oy, oz, ax, ay, az, px, py, pz)


def _read_object_list():
	""" Open the file specified in the object_config_file variable

	Read the list of component names"""

	scene_object_list = []
	print ("Reading '%s' config file for VIMAN poster" % object_config_file)

	try:
		fp = open(object_config_file, "r")
		for line in fp:
			match = re.search('object (\w+)', line)
			if match != None:
				scene_object_list.append(match.group(1))
				print ("\t- %s" % match.group(1))
		fp.close()
	except IOError as detail:
		print (detail)
		return None

	return scene_object_list
