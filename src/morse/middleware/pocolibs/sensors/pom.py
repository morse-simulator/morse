from morse.middleware.pocolibs.sensors.Gyro_Poster import ors_pom_poster

def init_extra_module(self, component_instance, function, function_name):
	""" Setup the middleware connection with this data

	Prepare the middleware to handle the serialised data as necessary.
	"""
	# Compose the name of the poster, based on the parent and module names
	component_name = component_instance.blender_obj.name
	parent_name = component_instance.robot_parent.blender_obj.name
	poster_name = 'pom_{0}_{1}_Poster'.format(parent_name, component_name)

	poster_id = init_pom_poster(self, component_instance, poster_name)
	if poster_id != None:
		print ("Pocolibs created poster '%s' of type pom" % poster_id)
		component_instance.output_functions.append(function)
		# Store the name of the port
		self._poster_dict[component_name] = poster_id



def init_pom_poster(self, component_instance, poster_name):
	""" Prepare the data for a pom poster """
	poster_id, ok = ors_pom_poster.init_data(poster_name)
	if ok == 0:
		print ("ERROR creating poster. This module may not work")
		return None

	print ("pom poster ID: {0}".format(poster_id))
	return poster_id


def write_pom(self, component_instance):
	""" Write the sensor position to a poaster

	The argument must be the instance to a morse gyroscope class. """
	# Compute the current time
	pom_date, t = self._compute_date()

	# Get the data from the gyroscope object
	robot = component_instance.robot_parent

	# Get the id of the poster already created
	poster_id = self._poster_dict[component_instance.blender_obj.name]
	ors_pom_poster.post_data(poster_id,
			robot.position_3d.x, robot.position_3d.y,
			robot.position_3d.z, robot.position_3d.yaw,
			robot.position_3d.pitch, robot.position_3d.roll,
			pom_date)
