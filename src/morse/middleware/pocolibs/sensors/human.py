import math
import re
import GameLogic
from morse.middleware.pocolibs.sensors.Human_Poster import ors_human_poster

def init_extra_module(self, component_instance, function):
	""" Setup the middleware connection with this data

	Prepare the middleware to handle the serialised data as necessary.
	"""
	# Compose the name of the poster, based on the parent and module names
	component_name = component_instance.blender_obj.name
	parent_name = component_instance.robot_parent.blender_obj.name
	poster_name = 'human_{0}_{1}'.format(parent_name, component_name)
	poster_id = init_human_poster(self, component_instance, poster_name)

	if poster_id != None:
		component_instance.output_functions.append(function)
		# Store the name of the port
		self._poster_dict[component_name] = poster_id


def init_human_poster(self, component_instance, poster_name):
	""" Prepare the data for a human poster """

	self.human_data = ors_human_poster.generate_human_struct()

	# Init the data structures used by this poster
	poster_id, ok = ors_human_poster.init_data(poster_name)
	if ok == 0:
		print ("ERROR creating poster. This module may not work")
		return None

	#else:
	print("HUMAN Poster '%s' created (ID: %d)" % (poster_name, poster_id))

	return poster_id


def write_human(self, component_instance):
	""" Write the posture of a human to a poster """
	# Get the id of the poster already created
	poster_id = self._poster_dict[component_instance.blender_obj.name]
	parent = component_instance.robot_parent

	scene = GameLogic.getCurrentScene()

	ors_viman_poster.set_visible (self.human_data, )

	posted = ors_viman_poster.post_human_poster(poster_id, self.human_data)
