from abc import ABCMeta, abstractmethod
import morse.helpers.transformation

class MorseObjectClass(object):
	""" Basic Class for all 3D objects (components) used in the simulation.
		Provides common attributes. """

	# Make this an abstract class
	__metaclass__ = ABCMeta

	def __init__ (self, obj, parent=None):
		""" Constructor method. """
		# Fill in the data sent as parameters
		self.blender_obj = obj
		self.robot_parent = parent

		# Define the position of sensors with respect
		#  to their robot parent
		# TODO: implement this using morse.helpers.transformation
		if not parent == None:
			self.relative_position = obj.getVectTo(parent.blender_obj)

		# Create an instance of the 3d transformation class
		self.position_3d = morse.helpers.transformation.Transformation3d(obj)

		# Dictionary to store the data used by each component
		self.local_data = {}
		# List that will hold the ordey of the dictionary keys
		self.data_keys = []

		# Define lists of dynamically added functions
		self.del_functions = []


	def __del__(self):
		""" Destructor method. """
		print ("%s: I'm dying!!" % self.blender_obj.name)
		# Call specific functions added to this object
		for function in self.del_functions:
			function(self)


	#@abstractmethod
	def action(self):
		""" Call the regular action function of the component.
		
		Can be redefined in some of the subclases (sensor and actuator).
		"""
		self.default_action()


	@abstractmethod
	def default_action():
		""" Base action performed by any object.

		This method should be implemented by all subclasses that
		will be instanced (GPS, v_Omega, ATRV, etc.).
		"""
		pass


	def print_data(self):
		""" Print the current position of the blender object. """
		for variable, data in self.local_data.items():
			res = variable + str(data) + " "
		print ("%s" % res)
