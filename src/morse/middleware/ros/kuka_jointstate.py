import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('sensor_msgs');
from sensor_msgs.msg import JointState
from morse.middleware.ros import ROSReader

class JointStateReader(ROSReader):

    def initalize(self):
        ROSReader.initalize(self, JointState)

    def update(self, message):
        """ Method called as soon as JointState messages are published on the specific topic """
        logger.debug("Received JointState names: %s on topic %s"%(message.name, self.topic_name))
        logger.debug("Received JointState positons: %s on topic %s"%(message.position, self.topic_name))
        logger.debug("Received JointState velocity: %s on topic %s"%(message.velocity, self.topic_name))

        self.data["kuka_1"] = message.position[0]
        self.data["kuka_2"] = message.position[1]
        self.data["kuka_3"] = message.position[2]
        self.data["kuka_4"] = message.position[3]
        self.data["kuka_5"] = message.position[4]
        self.data["kuka_6"] = message.position[5]
        self.data["kuka_7"] = message.position[6]
