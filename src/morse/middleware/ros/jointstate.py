from sensor_msgs.msg import JointState
from morse.middleware.sockets.jointstate import fill_missing_pr2_joints
from morse.middleware.ros import ROSPublisher

class JointStatePublisher(ROSPublisher):

    def initialize(self):
        ROSPublisher.initialize(self, JointState)

    def default(self, ci='unused'):
        """ Publish the data of the posture sensor as a ROS JointState message """
        js = JointState()
        js.header = self.get_ros_header()

        # collect name and positions of jointstates from sensor
        js.name = self.data.keys()
        js.position = self.data.values()
        # for now leaving out velocity and effort
        #js.velocity = [1, 1, 1, 1, 1, 1, 1]
        #js.effort = [50, 50, 50, 50, 50, 50, 50]

        self.publish(js)


class JointStatePR2Publisher(ROSPublisher):

    def initialize(self):
        ROSPublisher.initialize(self, JointState)

    def default(self, ci='unused'):
        """ Publish the data of the posture sensor as a ROS JointState message """
        js = JointState()
        js.header = self.get_ros_header()

        joints =  fill_missing_pr2_joints(self.data)
        js.name = joints.keys()
        js.position = joints.values()

        self.publish(js)
