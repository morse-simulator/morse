import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('geometry_msgs')
import mathutils
from geometry_msgs.msg import Pose
from morse.middleware.ros import ROSReader

class PoseReader(ROSReader):

    def initialize(self):
        ROSReader.initialize(self, Pose)

    def update(self, message):
        """ Method called as soon as Pose messages are published on the specific topic,
        store ``x``, ``y``, ``z`` and ``roll``, ``pitch``, ``yaw`` in ``local_data``"""
        self.data["x"] = message.position.x
        self.data["y"] = message.position.y
        self.data["z"] = message.position.z

        quaternion = mathutils.Quaternion(message.orientation)
        euler = quaternion.to_euler()
        self.data['roll'] = euler.x
        self.data['pitch'] = euler.y
        self.data['yaw'] = euler.z
