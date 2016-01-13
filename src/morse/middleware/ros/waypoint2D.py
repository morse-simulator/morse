import logging; logger = logging.getLogger("morse." + __name__)
from geometry_msgs.msg import Pose2D
from morse.middleware.ros import ROSSubscriber

class Pose2DReader(ROSSubscriber):
    """ Subscribe to a Pose2D topic and set ``x``, ``y``, ``z`` local data.
    This is designed to be used with the waypoint actuator.
    """
    ros_class = Pose2D

    def update(self, message):
        logger.debug("Received Pose2D: < %s, %s, %s > on topic %s"% \
                (message.x, message.y, message.theta, self.topic_name ))
        self.data["x"] = message.x
        self.data["y"] = message.y
        self.data["z"] = message.theta
