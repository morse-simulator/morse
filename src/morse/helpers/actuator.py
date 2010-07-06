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
		self.modified_data = []


	def action(self):
		""" Call the action functions that have been added to the list. """
		received = False
		status = False

		# First the input functions
		for function in self.input_functions:
			status = function(self)
			received = received or status

		if received:
			# Data modification functions
			for function in self.input_modifiers:
				self.modified_data = function(self)

			#self.local_data = self.modified_data
			i = 0
			for variable in self.data_keys:
				self.local_data[variable] = self.modified_data[i]
				i = i + 1

		# Call the regular action function of the component
		self.default_action()
