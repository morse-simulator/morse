from sensor_msgs.msg import JointState
from morse.middleware.sockets.jointstate import fill_missing_pr2_joints
from morse.middleware.ros import ROSPublisher

class JointStatePublisher(ROSPublisher):
    """ Publish the data of the posture sensor. """
    ros_class = JointState

    def default(self, ci='unused'):
        js = JointState()
        js.header = self.get_ros_header()

        # collect name and positions of jointstates from sensor
        js.name = list(self.data.keys())[1:]
        js.position = list(self.data.values())[1:]
        # for now leaving out velocity and effort
        #js.velocity = [1, 1, 1, 1, 1, 1, 1]
        #js.effort = [50, 50, 50, 50, 50, 50, 50]

        self.publish(js)


class JointStatePR2Publisher(ROSPublisher):
    """ Publish the data of the posture sensor after filling missing PR2 joints. """
    ros_class = JointState

    def default(self, ci='unused'):
        js = JointState()
        js.header = self.get_ros_header()

        joints =  fill_missing_pr2_joints(self.data)
        del joints['timestamp']
        js.name = joints.keys()
        js.position = joints.values()

        self.publish(js)
