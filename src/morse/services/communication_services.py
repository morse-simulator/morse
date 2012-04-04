import logging; logger = logging.getLogger("morse." + __name__)
logger.setLevel(logging.DEBUG)
import bge
from morse.core.exceptions import MorseRPCInvokationError
from morse.core.services import service
from morse.core import status
import morse.helpers.transformation 

def _robot_exists(robot):
    try:
        for obj, robot_instance in bge.logic.robotDict.items():
            if obj.name == robot:
                return robot_instance
    except KeyError:
        try:
            for obj, robot_instance in bge.logic.externalRobotDict.items():
                if obj.name == robot:
                    return robot_instance
        except KeyError:
            return None


@service(component = "communication")
def distance_and_view(robot1, robot2):
    """ Return the distance between the two robots, and a boolean which
    described if one can view the other. 
    """
    r1 = _robot_exists(robot1)
    r2 = _robot_exists(robot2)

    if (not r1):
        raise MorseRPCInvokationError(robot1 + " does not exist in the simulation ")
    if (not r2):
        raise MorseRPCInvokationError(robot2 + " does not exist in the simulation ")

    dist = r1.position_3d.distance(r2.position_3d)

    closest_obj = r1.blender_obj.rayCastTo(r2.blender_obj)

    return (dist, closest_obj == r2.blender_obj)

