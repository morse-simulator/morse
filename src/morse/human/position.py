import Blender
import GameLogic

# get the controller the script it attached to
controller = GameLogic.getCurrentController()

# get a list of sensors connected to the python controller
senList = controller.sensors

# get a list of actuators connected to the python controller
actList = controller.actuators

human = controller.owner

# get the keyboard sensor
sit_down_key = senList["sit_down"]

# get the actuators
sitdown = actList["sitdown"]
standup  = actList["standup"]


def applyPosition():
	# Sitdown
	if sit_down_key.positive == True and human['statusStandUp'] == True:
		controller.activate(sitdown)
		human['statusStandUp'] = False
		
	# Standup
	elif sit_down_key.positive == True and human['statusStandUp'] == False:
		controller.activate(standup)
		human['statusStandUp'] = True
		