from abc import ABCMeta, abstractmethod
import morse.helpers.object

class MorseSensorClass(morse.helpers.object.MorseObjectClass):
	""" Basic Class for all sensors

	Inherits from the base object class.
	"""

	# Make this an abstract class
	__metaclass__ = ABCMeta

	def __init__ (self, obj, parent=None):
		""" Constructor method. """
		# Call the constructor of the parent class
		super(MorseSensorClass, self).__init__(obj, parent)
		#super(self.__class__, self).__init__(obj, parent)

		# Define lists of dynamically added functions
		self.output_functions = []
		self.output_modifiers = []

		# Define dictionary for modified data
		self.modified_data = []

	#def __del__(self):
	#	""" Destructor method. """
	#	# Call the destructor of the parent class
	#	super(self.__class__,self).__del__(obj)


	def action(self):
		""" Call the action functions that have been added to the list. """
		# Update the component's position in the world
		self.position_3d.update(self.blender_obj)

		# Call the regular action function of the component
		self.default_action()

		# Make a copy of the data before modifications
		#self.modified_data = self.local_data
		i = 0
		for variable in self.data_keys:
			self.modified_data[i] = self.local_data[variable]
			i = i + 1

		# Data modification functions
		for function in self.output_modifiers:
			self.modified_data = function(self)

		# Lastly output functions
		for function in self.output_functions:
			function(self)

