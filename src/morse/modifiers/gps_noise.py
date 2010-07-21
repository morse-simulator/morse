import GameLogic
import morse.modifiers.gaussian

class MorseGPSNoiseClass(object):
	def __init__(self, obj, parent=None):
		self.blender_obj = obj


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

		if function_name == "noisify":
			component_instance.output_modifiers.append(function)
		else:
			print ("Unknown function name for GPS Noise modifier. Check component_config.py file.")



	def noisify(self, component_instance):
		for i in range(0, 2):
			component_instance.modified_data[i] = morse.modifiers.gaussian.gaussian(0.1, component_instance.modified_data[i])

		return component_instance.modified_data
