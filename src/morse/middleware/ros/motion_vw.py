from geometry_msgs.msg import Twist
from morse.middleware.ros import ROSSubscriber

class TwistReader(ROSSubscriber):
    """ Subscribe to a motion command and set ``v`` and ``w`` local data. """
    ros_class = Twist

    def update(self, message):
        self.data["v"] = message.linear.x
        self.data["w"] = message.angular.z # yaw
