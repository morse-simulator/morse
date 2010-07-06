from abc import ABCMeta, abstractmethod

class MorseMiddlewareClass(object):
	""" Basic Class for all middlewares

	Provides common attributes. """

	# Make this an abstract class
	__metaclass__ = ABCMeta

	def __init__ (self, obj, parent=None):
		""" Constructor method. """
		pass

	def __del__(self):
		""" Destructor method. """
		print ("%s: Middleware finishing" % self.blender_obj.name)


	@abstractmethod
	def register_component():
		""" Abstract model for the component binding method

		Implemented by all subclasses of MorseMiddlewareClass.
		"""
		pass
