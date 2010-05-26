import json
import GameLogic

class MorseJsonClass(object):
	""" Serialise the data using JSON. """
	
	def __init__(self, obj, parent=None):
		""" Initialize the network and connect to the yarp server."""
		self.blender_obj = obj
		#self.init_components()


	def __del__(self):
		""" Destructor method. """
		pass

	def json_serialise(self, component_instance):
		""" Convert the data stored in the object's message data
			into a single string field containing a json structure. """
		new_data = dict()
		# Convert the dictionary into a json string
		new_data['json_data'] = json.dumps(component_instance.send_data)
		return new_data
