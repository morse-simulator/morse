from abc import ABCMeta, abstractmethod
import morse.helpers.object

class MorseActuatorClass(morse.helpers.object.MorseObjectClass):
	""" Basic Class for all actuator ovjects.

	Provides common attributes. """

	# Make this an abstract class
	__metaclass__ = ABCMeta

	def __init__ (self, obj, parent=None):
		""" Constructor method. """
		# Call the constructor of the parent class
		super(MorseActuatorClass, self).__init__(obj, parent)
		#super(self.__class__, self).__init__(obj, parent)

		# Define lists of dynamically added functions
		self.input_functions = []
		self.input_modifiers = []

		# Define dictionary for modified data
		self.modified_data = {}


	def action(self):
		""" Call the action functions that have been added to the list. """
		# First the input functions
		for function in self.input_functions:
			function(self)

		self.modified_data = self.local_data

		# Data modification functions
		for function in self.input_modifiers:
			self.modified_data = function(self)

		# Call the regular action function of the component
		self.default_action()
