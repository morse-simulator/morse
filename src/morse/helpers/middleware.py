# Modules necessary to dynamically add methods to Middleware subclasses
import os
import sys
import re
import types

from abc import ABCMeta, abstractmethod

class MorseMiddlewareClass(object):
	""" Basic Class for all middlewares

	Provides common attributes. """

	# Make this an abstract class
	__metaclass__ = ABCMeta

	def __init__ (self, obj, parent=None):
		""" Constructor method. """
		self.blender_obj = obj
		self._extra_methods = []

	def __del__(self):
		""" Destructor method. """
		print ("%s: Middleware finishing" % self.blender_obj.name)


	@abstractmethod
	def register_component():
		""" Abstract model for the component binding method

		Implemented by all subclasses of MorseMiddlewareClass.
		"""
		pass


	def cleanup(self):
		""" Remove the modules linked dynamically """
		for module in self._extra_methods:
			delattr(self, module)


	def _check_function_exists(self, function_name):
		""" Checks if a class contains a specified function
		
		Returns a reference to the function, that can be used
		by other components
		"""
		try:
			# Get the reference to the function
			function = getattr(self, function_name)
			return function
		except AttributeError as detail:
			print ("ERROR: %s. Check the 'component_config.py' file for typos" % detail)
			return None


	def _add_method(self, source_file, function_name, component_instance):
		""" Include a new serialisation method in the middleware.

		Parameters are the file where the function is stored
		and the name of the function
		"""
		module_name = re.sub('/', '.', source_file)
		# Import the module containing the class
		try:
			__import__(module_name)
		except ImportError as detail:
			print ("ERROR: %s. Check the 'component_config.py' file for typos" % (detail))
			return
		module = sys.modules[module_name]

		try:
			# Get the reference to the new method
			func = getattr(module, function_name)
		except AttributeError as detail:
			print ("ERROR: %s, in extra module '%s'. Check the 'component_config.py' file for typos" % (detail, module_name))
			return

		# Insert the function and get a reference to it
		setattr(self, func.__name__, types.MethodType(func, self, self.__class__))
		bound_function = getattr(self, function_name)
		try:
			# Call the init method of the new serialisation
			module.init_extra_module(self, component_instance, bound_function)
		except AttributeError as detail:
			print ("ERROR: Method 'init_extra_module' not found in file '%s'" % source_file)

		# Store the name of the function, to cleanup later
		self._extra_methods.append(function_name)

		return bound_function
