import GameLogic
import morse.helpers.actuator

class PA10ActuatorClass(morse.helpers.actuator.MorseActuatorClass):
	""" Motion controller using linear and angular speeds

	This component will read an array of 6 floats, and apply them as
	rotation for the parts of the PA-10 arm.
	"""

	def __init__(self, obj, parent=None):
		print ('######## VW CONTROL INITIALIZATION ########')
		# Call the constructor of the parent class
		super(self.__class__,self).__init__(obj, parent)

		self.local_data['seg0'] = 0.0
		self.local_data['seg1'] = 0.0
		self.local_data['seg2'] = 0.0
		self.local_data['seg3'] = 0.0
		self.local_data['seg4'] = 0.0
		self.local_data['seg5'] = 0.0

		self.data_keys = ['seg0', 'seg1', 'seg2', 'seg3', 'seg4', 'seg5']

		# Initialise the copy of the data
		for variable in self.data_keys:
			self.modified_data.append(self.local_data[variable])

		# The axis along which the different segments rotate
		# Considering the rotation of the arm as installed in Jido
		self._dofs = ['z', 'y', 'y', 'z', 'y', 'z']

		self._segments = []
		segment = self.blender_obj.children[0]
		for i in range(6):
			self._segments.append(segment)
			segment = segment.children[0]

		print ('######## CONTROL INITIALIZED ########')



	def default_action(self):
		""" Apply (v, w) to the parent robot. """

		# Reset movement variables
		vx, vy, vz = 0.0, 0.0, 0.0
		rx, ry, rz = 0.0, 0.0, 0.0

		# Tick rate is the real measure of time in Blender.
		# By default it is set to 60, regardles of the FPS
		# If logic tick rate is 60, then: 1 second = 60 ticks
		ticks = GameLogic.getLogicTicRate()

		for i in range(6):
			# Scale the speeds to the time used by Blender

			key = ('seg%d' % i)
			try:
				rotation = self.local_data[key] / ticks
			# For the moment ignoring the division by zero
			# It happens apparently when the simulation starts
			except ZeroDivisionError:
				pass

			# Use the corresponding direction for each rotation
			if self._dofs[i] == 'y':
				ry = rotation
			elif self._dofs[i] == 'z':
				rz = rotation

			# Get the next segment
			segment = self._segments[i]

			# Give the movement instructions directly to the parent
			# The second parameter specifies a "local" movement
			segment.applyRotation([rx, ry, rz], True)

			# Reset the rotations for the next segment
			ry = rz = 0
