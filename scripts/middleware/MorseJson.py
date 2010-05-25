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

	def init_components(self):
		# Read the list of components and its associacted middleware
		# The file Component_Config.py is at the moment included
		#  in the .blend file of the scene
		import Component_Config
		# Add the hook functions to the appropriate components
		_component_list = Component_Config.component_modifier
		for component_name, modifier in _component_list.items():
			print ("Component: '%s' is operated by '%s'" % (component_name, modifier))
			instance = GameLogic.componentDict['OB' + component_name]
			# Add the yarp function to the component's action list
			instance.action_functions.append(self.json_serialise)


	def json_serialise(self, message_data, component_name):
		""" Convert the data stored in the object's message data
			into a single string field containing a json structure. """
		# Build a dictionary with the data in the variable message_data
		json_dict = dict()
		for element in message_data:
			variable_name, variable_data, variable_type = element[0], element[1], element[2]
			json_dict[variable_name] = variable_data
		# Convert the dictionary into a json string
		message = json.dumps(json_dict)
		message_data = [ ('json_data', message, 'string') ]
		return message_data
