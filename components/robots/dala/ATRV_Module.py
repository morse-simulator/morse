import GameLogic

# Definition of global variables
ob = ''

def init(contr):
	global ob

	motion_act = contr.actuators['motion_actuator']

	#sets the number of frames used to reach the target velocity
	motion_act.damping = 10

	#sets to move & rotate the object using the game object (local) axis.
	motion_act.useLocalDLoc = 1
	motion_act.useLocalDRot = 1

	# Get the current object
	contr= GameLogic.getCurrentController()
	ob = contr.owner

	# Get the dictionary for the robot's state
	robot_state_dict = GameLogic.robotDict[ob]

	# Add the speed values to the dictionary
	robot_state_dict['vx'] = 0.0
	robot_state_dict['vy'] = 0.0
	robot_state_dict['vz'] = 0.0

	# Add the rotation values to the dictionary
	robot_state_dict['rx'] = 0.0
	robot_state_dict['ry'] = 0.0
	robot_state_dict['rz'] = 0.0

	# Add movement status to the dictionary
	robot_state_dict['moveStatus'] = "Stop"


	print
	print '######## ATRV INITIALIZED ########'
	print


def move(contr):
	sensor = contr.sensors['Speed_msg']

	if sensor.positive:
		
		motion_act = contr.actuators['motion_actuator']
		
		# Get the dictionary for the robot's state
		robot_state_dict = GameLogic.robotDict[ob]
		
		# Get the speed values from the dictionary
		vx = robot_state_dict['vx']
		vy = robot_state_dict['vy']
		vz = robot_state_dict['vz']
		#print "{0} SETTING SPEED: {1}, {2}, {3}".format(ob.name, vx, vy, vz)
		
		# Get the rotation values from the dictionary
		rx = robot_state_dict['rx']
		ry = robot_state_dict['ry']
		rz = robot_state_dict['rz']
		
		motion_act.dLoc = (vx, vy, vz) #Blender  > 2.49
		motion_act.dRot = (rx, ry, rz) #Blender  > 2.49
		contr.activate(motion_act)
