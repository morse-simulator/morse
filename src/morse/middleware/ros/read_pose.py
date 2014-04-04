import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Pose, PoseStamped
from morse.middleware.ros import ROSSubscriber, mathutils

class PoseReader(ROSSubscriber):
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


class PoseStampedReader(ROSSubscriber):
    """ Subscribe to a PoseStamped topic and set ``x``, ``y``, ``z`` and ``roll``,
    ``pitch``, ``yaw`` local data.
    """
    ros_class = PoseStamped

    def update(self, message):
        self.data["x"] = message.pose.position.x
        self.data["y"] = message.pose.position.y
        self.data["z"] = message.pose.position.z

        quaternion = mathutils.Quaternion((message.pose.orientation.w,
                                           message.pose.orientation.x,
                                           message.pose.orientation.y,
                                           message.pose.orientation.z))
        euler = quaternion.to_euler()
        self.data['roll'] = euler.x
        self.data['pitch'] = euler.y
        self.data['yaw'] = euler.z
