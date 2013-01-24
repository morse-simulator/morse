import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Vector3
from morse.middleware.ros import ROSReader

class Vector3Reader(ROSReader):

    def initialize(self):
        ROSReader.initialize(self, Vector3)

    def update(self, message):
        """ Method called as soon as Vector3 messages are published on the specific topic """
        self.data["pan"] = message.y
        self.data["tilt"] = message.z
