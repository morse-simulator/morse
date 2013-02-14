import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Vector3
from morse.middleware.ros import ROSReader

class Vector3Reader(ROSReader):
    """ Subscribe to a Vector3 topic and set pan,tilt local data, according to
    the rotation axis (pan: y-axis, tilt: z-axis).
    """
    _type_name = "geometry_msgs/Vector3"

    def initialize(self):
        ROSReader.initialize(self, Vector3)

    def update(self, message):
        self.data["pan"] = message.y
        self.data["tilt"] = message.z
