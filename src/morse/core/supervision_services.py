import GameLogic
from morse.core.services import service
from morse.core import status

@service(component = "simulation")
def list_robots():
    return (status.SUCCESS, [obj.name for obj in GameLogic.robotDict.keys()])

