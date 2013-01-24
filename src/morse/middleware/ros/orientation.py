import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('geometry_msgs')
import math
import mathutils
from geometry_msgs.msg import Quaternion
from morse.middleware.ros import ROSReader

class QuaternionReader(ROSReader):

    def initialize(self):
        ROSReader.initialize(self, Quaternion)

    def update(self, message):
        """ Method called as soon as Quaternion messages are published on the specific topic """
        quaternion = mathutils.Quaternion(message)
        euler = quaternion.to_euler()
        self.data["roll"] = euler.x
        self.data["pitch"] = euler.y
        self.data["yaw"] = euler.z
        logger.debug("Set orientation to RPY (%.3f %.3f %.3f)" % \
                     tuple(math.degrees(a) for a in euler))
