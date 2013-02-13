import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Pose2D
from morse.middleware.ros import ROSReader

class Pose2DReader(ROSReader):

    def initialize(self):
        ROSReader.initialize(self, Pose2D)

    def update(self, message):
        """ Method called as soon as Pose2D messages are published on the specific
        topic, and stores values for ``x``, ``y`` and ``z`` in ``local_data``.
        This is designed to be used with the waypoint actuator """
        logger.debug("Received Pose2D: < %s, %s, %s > on topic %s"% \
                (message.x, message.y, message.theta, self.topic_name ))
        self.data["x"] = message.x
        self.data["y"] = message.y
        self.data["z"] = message.theta
