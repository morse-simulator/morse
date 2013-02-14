import roslib; roslib.load_manifest('std_msgs')
from std_msgs.msg import Bool
from morse.middleware.ros import ROSReader

class BoolReader(ROSReader):
    """ Subscribe to a boolean topic to control if we must or not emit light. """
    _type_name = "std_msgs/Bool"

    def initialize(self):
        ROSReader.initialize(self, Bool)

    def update(self, message):
        self.data["emit"] = message.data
