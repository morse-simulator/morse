import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Point
from morse.middleware.ros import ROSReader

class PointReader(ROSReader):

    def initalize(self):
        ROSReader.initalize(self, Point)

    def update(self, message):
        """ Method called as soon as Point messages are published on the specific topic """
        self.data["x"] = message.x
        self.data["y"] = message.y
        self.data["z"] = message.z
