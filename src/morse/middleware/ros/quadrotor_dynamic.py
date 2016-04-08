from std_msgs.msg import Float32MultiArray
from morse.middleware.ros import ROSSubscriber

class QuadrotorDynamicReader(ROSSubscriber):
    """ Subscribe to a float array topic and set quadrotor engines local data. """
    ros_class = Float32MultiArray

    def update(self, message):
        self.data["engines"] = message.data
