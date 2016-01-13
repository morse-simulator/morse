import logging; logger = logging.getLogger("morse." + __name__)
from geometry_msgs.msg import Wrench
from morse.middleware.ros import ROSSubscriber

class WrenchReader(ROSSubscriber):
    """ Subscribe to a Wrench topic and set force and torque (x,y,z) local data. """
    ros_class = Wrench

    def update(self, message):
        self.data["force"][0] = message.force.x
        self.data["force"][1] = message.force.y
        self.data["force"][2] = message.force.z
        self.data["torque"][0] = message.torque.x
        self.data["torque"][1] = message.torque.y
        self.data["torque"][2] = message.torque.z
        logger.debug("Applying force: [%s, %s, %s], torque: [%s, %s, %s]"%
                     (message.force.x, message.force.y, message.force.z,
                      message.torque.x, message.torque.y, message.torque.z))
