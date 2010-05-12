import GameLogic

class Morse_Object_Class(object):
	""" Basic Class for all 3D objects (components) used in the simulation.
		Provides common attributes. """

	def __init__ (self, obj, parent=None):
		""" Constructor method. """
		# Fill in the data sent as parameters
		self.blender_obj = obj
		self.position = obj.position
		self.rotation = [0.0, 0.0, 0.0]
		self.robot_parent = parent

		# Define lists of dynamically added functions
		self.action_functions = []
		self.del_functions = []

		print ("Instance of 'Morse_Object' created for %s" % obj.name)

	def __del__(self):
		""" Destructor method. """
		print ("%s: I'm dying!!" % self.blender_obj.name)
		# Call specific functions added to this object
		for function in self.del_functions:
			function()

	def action(self):
		""" Call the action functions that have been added to the list. """
		self.default_action()
		for function in self.action_functions:
			function()

	def print_position(self):
		""" Print the current position of the blender object. """
		print ("%s is at %s" % (self.blender_obj.name, self.position))
