import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Twist
from morse.middleware.ros import ROSReader

class TwistReader(ROSReader):

    def initalize(self):
        ROSReader.initalize(self, Twist)

    def update(self, message):
        """ Method called as soon as Twist messages are published on the specific topic """
        self.data["v"] = message.linear.x
        self.data["w"] = message.angular.z # yaw
