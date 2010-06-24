import GameLogic
import morse.helpers.object

class DestinationActuatorClass(morse.helpers.object.MORSEObjectClass):
	""" Destination motion controller

	This controller will receive a destination point and
	make the robot move to that location by moving without turning.
	"""

	def __init__(self, obj, parent=None):

		print ('######## CONTROL INITIALIZATION ########')
		# Call the constructor of the parent class
		super(self.__class__,self).__init__(obj, parent)

		self.tolerance = 0.5
		self.speed = 5.0
		self.destination = [0.0, 0.0, 0.0]

		#self.local_data['speed'] = 0.0
		self.local_data['x'] = 0.0
		self.local_data['y'] = 0.0
		self.local_data['z'] = 0.0

		print ('######## CONTROL INITIALIZED ########')


	def default_action(self):
		""" Move the object towards the destination. """

		parent = self.robot_parent
		parent.move_status = "Transit"

###### OLD STUFF  #####

		# Get the orientation of the robot
		try:
			if parent.blender_obj['Orientation'] == 'NED':
				NED = True
		except KeyError as detail:
			NED = False


		"""
		# THIS HAS TO BE DONE WITH A REQUEST
		########################### SPEED ###############################
		# Retrieve the port we want to read from
		sp = GameLogic.orsConnector.getPort(speed_port_name)

		#non-blocking read of the port
		speed_msg = sp.read(False)

		if speed_msg!=None:
			robot_state_dict['speed'] = speed_msg.get(0).asDouble()
			print ("SETTING SPEED TO: {0}".format(speed))
		"""

		self.destination = [ self.local_data['x'], self.local_data['y'], self.local_data['z'] ]

		#print ("STRAIGHT GOT DESTINATION: {0}".format(destination))
		#print ("Robot {0} move status: '{1}'".format(parent.blender_obj.name, parent.move_status))

		"""
		if NED == True:
			d=destination[0]
			destination[0] = destination[1]
			destination[1] = d				
			destination[2] = -destination[2]
		"""


		"""
		# DON"T KNOW IF THIS IS NECESSARY NOW
		try:
			# Exit the function if there has been no command to move
			if not robot_state_dict['moveStatus'] == "Transit":
				return
		except KeyError:
			# Also exit if there is no moveStatus property
			return
		"""

		distance, global_vector, local_vector = self.blender_obj.getVectTo(destination)

		#print ("GOT DISTANCE: {0}".format(distance))

		if distance > 0:
			global_vector.normalize()


			# Tick rate is the real measure of time in Blender.
			# By default it is set to 60, regardles of the FPS
			# If logic tick rate is 60, then: 1 second = 60 ticks
			ticks = GameLogic.getLogicTicRate()
	
			if NED == True:
				# Scale the speeds to the time used by Blender
				try:
					vx = global_vector[1] * speed / ticks
					vy = global_vector[0] * speed / ticks
					vz = -global_vector[2] * speed / ticks
				# For the moment ignoring the division by zero
				# It happens apparently when the simulation starts
				except ZeroDivisionError:
					pass
			else:
				# Scale the speeds to the time used by Blender
				try:
					vx = global_vector[0] * speed / ticks
					vy = global_vector[1] * speed / ticks
					vz = global_vector[2] * speed / ticks
				# For the moment ignoring the division by zero
				# It happens apparently when the simulation starts
				except ZeroDivisionError:
					pass

		# If the target has been reached, change the status
		elif distance <= self.tolerance:
			# Reset movement variables
			vx, vy, vz = 0.0, 0.0, 0.0
			rx, ry, rz = 0.0, 0.0, 0.0

			parent.move_status = "Stop"
			print ("TARGET REACHED")
			print ("Robot {0} move status: '{1}'".format(parent.blender_obj.name, parent.move_status))

		# Give the movement instructions directly to the parent
		# The second parameter specifies a "local" movement
		parent.applyMovement([vx, vy, vz], True)
		parent.applyRotation([rx, ry, rz], True)


