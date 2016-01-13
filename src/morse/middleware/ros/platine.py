from geometry_msgs.msg import Vector3
from morse.middleware.ros import ROSSubscriber

class Vector3Reader(ROSSubscriber):
    """ Subscribe to a Vector3 topic and set pan,tilt local data, according to
    the rotation axis (pan: y-axis, tilt: z-axis).
    """
    ros_class = Vector3

    def update(self, message):
        self.data["pan"] = message.y
        self.data["tilt"] = message.z
