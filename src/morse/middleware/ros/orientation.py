import logging; logger = logging.getLogger("morse." + __name__)
import math
from geometry_msgs.msg import Quaternion
from morse.middleware.ros import ROSSubscriber, mathutils

class QuaternionReader(ROSSubscriber):
    """ Subscribe to a Quaternion topic and set roll,pitch,yaw local data. """
    ros_class = Quaternion

    def update(self, message):
        quaternion = mathutils.Quaternion((message.w, message.x, message.y, message.z))
        euler = quaternion.to_euler()
        self.data["roll"] = euler.x
        self.data["pitch"] = euler.y
        self.data["yaw"] = euler.z
        logger.debug("Set orientation to RPY (%.3f %.3f %.3f)" % \
                     tuple(math.degrees(a) for a in euler))
