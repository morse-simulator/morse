from std_msgs.msg import Float32MultiArray
from morse.middleware.ros import ROSSubscriber

class RotorcraftAttitudeReader(ROSSubscriber):
    """ Subscribe to a float array topic and set quadrotor attitude controller local data. """
    ros_class = Float32MultiArray

    def update(self, message):
        self.data["roll"]   = message.data[0]
        self.data["pitch"]  = message.data[1]
        self.data["yaw"]    = message.data[2]
        self.data["thrust"] = message.data[3]
