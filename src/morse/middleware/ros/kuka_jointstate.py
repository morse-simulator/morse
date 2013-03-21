import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('sensor_msgs');
from sensor_msgs.msg import JointState
from morse.middleware.ros import ROSReader

class JointStateReader(ROSReader):
    """ Subscribe to a JointState topic and set kuka_{1-7} to the position[0-6]. """
    ros_class = JointState

    def update(self, message):
        logger.debug("Received JointState names: %s on topic %s"%(message.name, self.topic_name))
        logger.debug("Received JointState positons: %s on topic %s"%(message.position, self.topic_name))
        logger.debug("Received JointState velocity: %s on topic %s"%(message.velocity, self.topic_name))

        for i in range(7):
            self.data["kuka_%i"%(i+1)] = message.position[i]

