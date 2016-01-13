import logging; logger = logging.getLogger("morse." + __name__)
from geometry_msgs.msg import Twist
from morse.middleware.ros import ROSSubscriber

class TwistReader(ROSSubscriber):
    """ Subscribe to a motion command and set ``x``, ``y`` and ``w`` local data. """
    ros_class = Twist

    def update(self, message):
        self.data["x"] = message.linear.x
        self.data["y"] = message.linear.y
        self.data["w"] = message.angular.z # yaw
        logger.debug("Executing x,y,omega movement: <%s, %s, %s>"%
                     (message.linear.x, message.linear.y, message.angular.z))
