import GameLogic
from morse.core.services import service

@service(component = "simulation")
def list_robots():
    return [obj.name for obj in GameLogic.robotDict.keys()]

