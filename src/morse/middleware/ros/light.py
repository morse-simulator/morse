import roslib; roslib.load_manifest('std_msgs')
from std_msgs.msg import Bool
from morse.middleware.ros import ROSReader

class BoolReader(ROSReader):

    def initalize(self):
        ROSReader.initalize(self, Bool)

    def update(self, message):
        """ Method called as soon as Bool messages are published on the specific topic """
        self.data["emit"] = message.data
