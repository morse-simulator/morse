import GameLogic
import GameKeys

def move(contr):
	obj = contr.owner
	parent = obj.parent
	# Get the dictionary for the robot's state
	robot_state_dict = GameLogic.robotDict[parent]


	keys_sensor = GameLogic.getCurrentController().sensors['keys_sensor']	
	#pressed_keys = keys_sensor.getPressedKeys()
	pressed_keys = keys_sensor.events

	msg_act = contr.actuators['Send_update_msg']

	"""
	parent_state_dict = GameLogic.robotDict[parent]
	parent_actuator = parent_state_dict['motion_actuator']
	parent_controller = parent_state_dict['motion_controller']
	"""


	for key, status in pressed_keys:
		#print "GOT: {0}, STATUS {1}".format(key, status)
		if key == GameKeys.UPARROWKEY:
			msg_act.propName = parent.name
			msg_act.subject = 'Speed'
			robot_state_dict['vx'] = robot_state_dict['vx'] + 0.01		
			contr.activate(msg_act)
			
		if key == GameKeys.DOWNARROWKEY:
			msg_act.propName = parent.name
			msg_act.subject = 'Speed'
			robot_state_dict['vx'] = robot_state_dict['vx'] - 0.01		
			contr.activate(msg_act)

		if key == GameKeys.LEFTARROWKEY:
			msg_act.propName = parent.name
			msg_act.subject = 'Speed'
			robot_state_dict['vy'] = robot_state_dict['vy'] + 0.01		
			contr.activate(msg_act)

		if key == GameKeys.RIGHTARROWKEY:
			msg_act.propName = parent.name
			msg_act.subject = 'Speed'
			robot_state_dict['vy'] = robot_state_dict['vy'] - 0.01		
			contr.activate(msg_act)
