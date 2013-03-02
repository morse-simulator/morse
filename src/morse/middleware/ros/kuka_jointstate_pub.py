import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('sensor_msgs');
from sensor_msgs.msg import JointState
from morse.middleware.ros import ROSPublisher

class JointStatePublisher(ROSPublisher):
    """ Publishes a JointState topic and set kuka_{1-7} to the position[0-6]. """
    ros_class = JointState

    def default(self, ci='unused'):
        message = JointState()
        message.name = [''] * 7
        message.position = [0] * 7
        message.velocity = [0] * 7
        message.effort = [0] * 7
        for i in range(7):
            message.name[i] = "arm_%i_joint"%i
            message.position[i] = self.data["kuka_%i"%(i+1)]

        self.publish(message)

