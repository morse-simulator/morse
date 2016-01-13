from std_msgs.msg import Bool
from morse.middleware.ros import ROSSubscriber

class BoolReader(ROSSubscriber):
    """ Subscribe to a boolean topic to control if we must or not emit light. """
    ros_class = Bool

    def update(self, message):
        self.data["emit"] = message.data
