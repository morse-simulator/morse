import roslib; roslib.load_manifest('std_msgs')
from std_msgs.msg import Float32
from morse.middleware.ros import ROSPublisher

class Float32Publisher(ROSPublisher):

    def initialize(self):
        ROSPublisher.initialize(self, Float32)

    def default(self, ci='unused'):
        """ Publish the data of the battery sensor as a single Float32 message """
        msg = Float32()
        msg.data = self.data['charge']

        self.publish(msg)
