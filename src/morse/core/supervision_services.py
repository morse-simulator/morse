import logging; logger = logging.getLogger("morse." + __name__)
import GameLogic
from morse.core.services import service
from morse.core import status
from morse.blender.main import reset_objects as main_reset

@service(component = "simulation")
def list_robots():
    """ Return a list of the robots in the current scenario

    Uses the list generated during the initialisation of the scenario
    """
    return [obj.name for obj in GameLogic.robotDict.keys()]

@service(component = "simulation")
def reset_objects():
    """ Restore all simulation objects to their original position

    Upon receiving the request using sockets, call the
    'reset_objects' function located in morse/blender/main.py
    """
    contr = GameLogic.getCurrentController()
    main_reset(contr)
    return "Objects restored to initial position"
