import logging; logger = logging.getLogger("morse." + __name__)
from geometry_msgs.msg import Pose, PoseStamped
from morse.middleware.ros import ROSSubscriber, mathutils

class PoseReader(ROSSubscriber):
    """ Subscribe to a Pose topic and set ``position`` as Vector and ``x``, ``y``, ``z``
    and ``orientation`` as Quaternion and ``roll``, ``pitch``, ``yaw`` local data.
    """
    ros_class = Pose

    def update(self, message):
        self.data["x"] = message.position.x
        self.data["y"] = message.position.y
        self.data["z"] = message.position.z
        self.data['position'] = mathutils.Vector((message.position.x, message.position.y, message.position.z))

        quaternion = mathutils.Quaternion((message.orientation.w,
                                           message.orientation.x,
                                           message.orientation.y,
                                           message.orientation.z))
        self.data['orientation'] = quaternion
        euler = quaternion.to_euler()
        self.data['roll'] = euler.x
        self.data['pitch'] = euler.y
        self.data['yaw'] = euler.z


class PoseStampedReader(ROSSubscriber):
    """ Subscribe to a PoseStamped topic and set ``position`` as Vector and ``x``, ``y``, ``z``
    and ``orientation`` as Quaternion and ``roll``, ``pitch``, ``yaw`` local data.
    """
    ros_class = PoseStamped

    def update(self, message):
        self.data["x"] = message.pose.position.x
        self.data["y"] = message.pose.position.y
        self.data["z"] = message.pose.position.z
        self.data['position'] = mathutils.Vector((message.pose.position.x, message.pose.position.y, message.pose.position.z))

        quaternion = mathutils.Quaternion((message.pose.orientation.w,
                                           message.pose.orientation.x,
                                           message.pose.orientation.y,
                                           message.pose.orientation.z))
        self.data['orientation'] = quaternion
        euler = quaternion.to_euler()
        self.data['roll'] = euler.x
        self.data['pitch'] = euler.y
        self.data['yaw'] = euler.z

class PoseToQueueReader(PoseReader):
    def update(self, message):
        super().update(message)
        mat_rot = self.data['orientation'].to_matrix()
        mat_loc = mathutils.Matrix.Translation(self.data['position'])
        self.data['pose_queue'].put(mat_loc * mat_rot.to_4x4())
