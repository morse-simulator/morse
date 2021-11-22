from rosgraph_msgs.msg import Clock
from morse.middleware.ros import ROSPublisher

class ClockPublisher(ROSPublisher):
    """ Publish the simulator clock. """
    ros_class = Clock

    def initialize(self):
        ROSPublisher.initialize(self)

    def default(self, ci='unused'):
        msg = Clock()
        msg.clock = self.get_time()

        self.publish(msg)
