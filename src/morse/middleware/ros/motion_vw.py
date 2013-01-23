import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Twist
from morse.middleware.ros import ROSReader

class TwistReader(ROSReader):

    def initalize(self):
        ROSReader.initalize(self, Twist)

    def update(self, message):
        self.component_instance.local_data["v"] = message.linear.x
        self.component_instance.local_data["w"] = message.angular.z # yaw
