import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Pose
from morse.middleware.ros import ROSReader, mathutils

class PoseReader(ROSReader):
    """ Subscribe to a Pose topic and set ``x``, ``y``, ``z`` and ``roll``,
    ``pitch``, ``yaw`` local data.
    """
    ros_class = Pose

    def update(self, message):
        self.data["x"] = message.position.x
        self.data["y"] = message.position.y
        self.data["z"] = message.position.z

        quaternion = mathutils.Quaternion((message.orientation.w, message.orientation.x, message.orientation.y, message.orientation.z))
        euler = quaternion.to_euler()
        self.data['roll'] = euler.x
        self.data['pitch'] = euler.y
        self.data['yaw'] = euler.z
