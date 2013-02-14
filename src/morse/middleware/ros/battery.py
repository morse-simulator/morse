import roslib; roslib.load_manifest('std_msgs')
from std_msgs.msg import Float32
from morse.middleware.ros import ROSPublisher

class Float32Publisher(ROSPublisher):
    """ Publish the charge of the battery sensor. """
    _type_name = "std_msgs/Float32"

    def initialize(self):
        ROSPublisher.initialize(self, Float32)

    def default(self, ci='unused'):
        msg = Float32()
        msg.data = self.data['charge']

        self.publish(msg)
