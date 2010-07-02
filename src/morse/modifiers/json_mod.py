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


	def register_component(self, component_name, component_instance, mod_data):
		""" Add the corresponding function to a component. """
		# Extract the information for this modifier
		# This will be tailored for each middleware according to its needs
		function_name = mod_data[1]

		try:
			# Get the reference to the function
			function = getattr(self, function_name)
		except AttributeError as detail:
			print ("ERROR: %s. Check the 'component_config.py' file for typos" % detail)
			return

		# Choose what to do, depending on the function being used
		# Data read functions
		if function_name == "json_decode":
			component_instance.input_modifiers.append(function)
		# Data write functions
		elif function_name == "json_encode":
			component_instance.output_modifiers.append(function)
		else:
			print ("Unknown function name for JSON modifier. Check component_config.py file.")



	def json_encode(self, component_instance):
		""" Convert a dictionary into a JSON string.

		Convert the data stored in the object's message data
		into a single string field containing a json structure. """
		new_data = dict()
		# Convert the dictionary into a json string
		new_data['json_data'] = json.dumps(component_instance.modified_data)
		return new_data


	def json_decode(self, component_instance):
		""" Convert a JSON string into a dictionary. """
		new_data = dict()
		# Convert the dictionary into a json string
		new_data = json.loads(component_instance.modified_data)
		return new_data['json_data']
