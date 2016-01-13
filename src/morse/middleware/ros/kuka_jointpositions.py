import logging; logger = logging.getLogger("morse." + __name__)
from motion_control_msgs.msg import JointPositions
from morse.middleware.ros import ROSSubscriber

class JointPositionsReader(ROSSubscriber):

    ros_class = JointPositions

    def update(self, msg):

        for i in range(7):
            self.data["kuka_%i"%(i+1)] = msg.positions[i]

