from geometry_msgs.msg import Point
from morse.middleware.ros import ROSSubscriber

class PointReader(ROSSubscriber):
    """ Subscribe to a Point topic and set x,y,z local data. """
    ros_class = Point

    def update(self, message):
        self.data["x"] = message.x
        self.data["y"] = message.y
        self.data["z"] = message.z
