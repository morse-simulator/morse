import logging; logger = logging.getLogger("morse." + __name__)
#from geometry_msgs.msg import Twist
from morse_uuv_msgs.msg import ThrusterControl
from morse.middleware.ros import ROSSubscriber

class CtrlReader(ROSSubscriber):
    """ Subscribe to a motion command and set ``v`` and ``w`` local data. """
    ros_class = ThrusterControl
    topic_name = "SOMETHING"

    def update(self, message):
        self.data['desired_rudder'] = message.rudder
        self.data['desired_elevator'] = -message.elevator 
        self.data['desired_thrust'] = message.thrust
