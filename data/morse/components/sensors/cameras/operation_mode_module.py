import GameLogic

def select(contr):
	""" Change the Game Engine State to the one
		defined in the 'Operation_Mode' property of the object.
		That variable should be set via the GUI configuration"""

	ob = contr.owner
	mode = ob['Operation_Mode']

	state_act = contr.actuators['State_Selector']
	state_act.mask = mode
	contr.activate(state_act)
