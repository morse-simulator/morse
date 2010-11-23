import GameLogic

# get the controller the script it attached to
contr = GameLogic.getCurrentController()
human = contr.owner

# get the keyboard sensor
sit_down_key = contr.sensors["sit_down"]

# get the actuators
sitdown = contr.actuators["sitdown"]
standup = contr.actuators["standup"]


def applyPosition():
    """ Change the stance of the human model

    Make the human sit down or stand up, using a preset animation.
    """
    # Sitdown
    if sit_down_key.positive == True and human['statusStandUp'] == True:
        contr.activate(sitdown)
        human['statusStandUp'] = False

    # Standup
    elif sit_down_key.positive == True and human['statusStandUp'] == False:
        contr.activate(standup)
        human['statusStandUp'] = True
