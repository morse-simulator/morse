import logging; logger = logging.getLogger("morse." + __name__)

from morse.core.services import interruptible
from morse.middleware.ros_request_manager import ros_action
from morse.core.overlay import MorseOverlay
from morse.core import status

from morse.middleware.ros.helpers import ros_add_to_syspath
ros_add_to_syspath("move_base_msgs")
from move_base_msgs.msg import *

class WayPoint(MorseOverlay):

    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(overlaid_object)

    def move_base_on_completion(self, result):
        state, value = result

        logger.info("MoveBase completed! got value " + str(value))

        return (state, MoveBaseResult())

    @interruptible
    @ros_action(type = MoveBaseAction)
    def move_base(self, req):
        logger.info("Going to (%d, %d)" % (req.target_pose.pose.position.x, req.target_pose.pose.position.y))

        self.overlaid_object.goto(
                self.chain_callback(self.move_base_on_completion),
                req.target_pose.pose.position.x,
                req.target_pose.pose.position.y,
                req.target_pose.pose.position.z)

