import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('motion_control_msgs');
from motion_control_msgs.msg import JointPositions
from morse.middleware.ros import ROSReader

class JointPositionsReader(ROSReader):

    ros_class = JointPositions

    def update(self, msg):

        for i in range(7):
            self.data["kuka_%i"%(i+1)] = msg.positions[i]

