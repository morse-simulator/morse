import json
import GameLogic

class MorseUTMClass(object):
	""" Convert between Blender and UTM coordinates. """
	
	def __init__(self, obj, parent=None):
		""" Initialize the network and connect to the yarp server."""
		self.blender_obj = obj
		#self.init_components()


	def __del__(self):
		""" Destructor method. """
		pass

	def blender_to_utm(self, component_instance):
		""" Convert the coordinates from Blender to UTM reference. """
		new_data = dict()
		# Convert the dictionary into a json string
		new_data['json_data'] = json.dumps(component_instance.send_data)
		return new_data

	def utm_to_blender(self, component_instance):
		""" Convert the coordinates from UTM to Blender reference. """
