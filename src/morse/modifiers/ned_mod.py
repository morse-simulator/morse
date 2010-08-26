import GameLogic

class MorseNEDClass(object):
	""" Convert between ENU and NED coordinates. """
	
	def __init__(self, obj, parent=None):
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
		if function_name == "ned_to_blender":
			component_instance.input_modifiers.append(function)
		# Data write functions
		elif function_name == "blender_to_ned":
			component_instance.output_modifiers.append(function)
		
		# Data read functions
		if function_name == "ned_angle_to_blender":
			component_instance.input_modifiers.append(function)
		# Data write functions
		elif function_name == "blender_to_ned_angle":
			component_instance.output_modifiers.append(function)

	def blender_to_ned(self, component_instance):
		""" Convert the coordinates from Blender to UTM reference. """
		tmp = component_instance.modified_data[0]
		component_instance.modified_data[0] = component_instance.modified_data[1]
		component_instance.modified_data[1] = tmp
		component_instance.modified_data[2] = -component_instance.modified_data[2]

		return component_instance.modified_data


	def ned_to_blender(self, component_instance):
		""" Convert the coordinates from UTM to Blender reference. """
		tmp = component_instance.modified_data[0]
		component_instance.modified_data[0] = component_instance.modified_data[1]
		component_instance.modified_data[1] = tmp
		component_instance.modified_data[2] = -component_instance.modified_data[2]

		return component_instance.modified_data



	def blender_to_ned_angle(self, component_instance):
		""" Convert the coordinates from Blender to UTM reference. """
		yaw = 90-component_instance.modified_data[0]
		component_instance.modified_data[0] = component_instance.modified_data[2]
		component_instance.modified_data[1] =-component_instance.modified_data[1]
		component_instance.modified_data[2] = yaw

		return component_instance.modified_data


	def ned_angle_to_blender(self, component_instance):
		""" Convert the coordinates from UTM to Blender reference. """
		yaw = 90.0-component_instance.modified_data[2]
		component_instance.modified_data[0] = yaw
		component_instance.modified_data[1] =-component_instance.modified_data[1]
		component_instance.modified_data[2] = component_instance.modified_data[0]

		return component_instance.modified_data
