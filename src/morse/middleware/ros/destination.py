import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Point
from morse.middleware.ros import ROSReader

class PointReader(ROSReader):
    """ Subscribe to a Point topic and set x,y,z local data. """
    _type_name = "geometry_msgs/Point"

    def initialize(self):
        ROSReader.initialize(self, Point)

    def update(self, message):
        self.data["x"] = message.x
        self.data["y"] = message.y
        self.data["z"] = message.z
