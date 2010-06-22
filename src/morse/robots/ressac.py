import GameLogic
import morse.helpers.object


class RessacClass(morse.helpers.object.MorseObjectClass):
	""" Class definition for the Ressac.
		Sub class of Morse_Object. """

	def __init__(self, obj, parent=None):
		""" Constructor method.
			Receives the reference to the Blender object.
			Optionally it gets the name of the object's parent,
			but that information is not currently used for a robot. """
		# Call the constructor of the parent class
		print ("######## ROBOT '%s' INITIALIZING ########" % obj.name)
		super(self.__class__,self).__init__(obj, parent)

		# Add the variable move_status to the object
		self.move_status = "Stop"

		print ('######## ROBOT INITIALIZED ########')

	def default_action(self):
		""" Main function of this component. """
		pass
