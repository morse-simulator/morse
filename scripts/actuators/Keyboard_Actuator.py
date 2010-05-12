import GameLogic
import GameKeys
import Morse_Object

class Keyboard_Actuator_Class(Morse_Object.Morse_Object_Class):
	""" Class definition for a motion controller that changes speed
		and rotation of the robot using the keyboard arrows. """

	def __init__(self, obj, parent=None):
		print ('######## CONTROL INITIALIZATION ########')
		# Call the constructor of the parent class
		super(self.__class__,self).__init__(obj, parent)

		self.speed = 0.05
		print ('######## CONTROL INITIALIZED ########')



	def default_action(self):
		""" Interpret keyboard presses and assign them to movement
			for the robot."""
		keys_sensor = GameLogic.getCurrentController().sensors['keys_sensor']
		#pressed_keys = keys_sensor.getPressedKeys()
		pressed_keys = keys_sensor.events

		# Reset movement variables
		vx, vy, vz = 0.0, 0.0, 0.0
		rx, ry, rz = 0.0, 0.0, 0.0

		for key, status in pressed_keys:
			#print ("GOT: {0}, STATUS {1}".format(key, status))
			if key == GameKeys.UPARROWKEY:
				vx = self.speed

			if key == GameKeys.DOWNARROWKEY:
				vx = -self.speed

			if key == GameKeys.LEFTARROWKEY:
				rz = self.speed / 2.0

			if key == GameKeys.RIGHTARROWKEY:
				rz = -self.speed / 2.0

		# Get the Blender object of the parent robot
		parent = self.robot_parent.blender_obj

		# Give the movement instructions directly to the parent
		# The second parameter specifies a "local" movement
		parent.applyMovement([vx, vy, vz], True)
		parent.applyRotation([rx, ry, rz], True)
