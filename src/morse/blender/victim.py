import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic

def init(contr):
    """ Set the mesh color to red """
    obj = contr.owner
    obj.color = [1.0, 0.5, 0.5, 1.0]


def heal(contr):
    """ Change the status of the victim
    
    Change the material to a green color,
    and the status to healed.
    """
    obj = contr.owner

    if obj['Severity'] > 0:
        obj['Severity'] = obj['Severity'] -1
        # Set the colors depending on the severity of the injuries
        red = 1 - obj['Severity'] * 0.05
        green = 0.5 + red
        obj.color = [red, green, 0.5, 1.0]

    # When fully healed, mark as not injured
    if obj['Severity'] == 0:
        obj['Injured'] = False
