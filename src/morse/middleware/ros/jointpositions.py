import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('motion_control_msgs');
from motion_control_msgs.msg import JointPositions
from morse.middleware.ros import ROSReader

class JointPositionsReader(ROSReader):

    ros_class = JointPositions

    def update(self, msg):

        joint_names = msg.names
       
        for joint in joint_names:
                # update the joint values of each of the joints
                self.data[joint] = msg.positions[joint]
