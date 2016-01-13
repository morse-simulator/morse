import logging; logger = logging.getLogger("morse." + __name__)
from geometry_msgs.msg import Twist
from morse.middleware.ros import ROSSubscriber

class TwistReader(ROSSubscriber):
    """ Subscribe to a Twist topic and set ``vx``, ``vy``, ``vz`` and ``vroll``,
    ``vpitch``, ``vyaw`` local data.
    """
    ros_class = Twist

    def update(self, message):
        self.data["vx"] = message.linear.x
        self.data["vy"] = message.linear.y
        self.data["vz"] = message.linear.z

        self.data['vroll'] = message.angular.x
        self.data['vpitch'] = message.angular.y
        self.data['vyaw'] = message.angular.z
