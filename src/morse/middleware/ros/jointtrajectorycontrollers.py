from control_msgs.msg import JointTrajectoryControllerState
from morse.middleware.ros import ROSPublisher

class JointTrajectoryControllerStatePublisher(ROSPublisher):
    """ Publish the data of the pr2 joint sensor. """
    ros_class = JointTrajectoryControllerState

    def default(self, ci='unused'):
        js = JointTrajectoryControllerState()
        js.header = self.get_ros_header()

        # timestamp is not a joint
        joints = dict(self.data)
        del joints['timestamp']
        js.joint_names = joints.keys()
        js.actual.positions = joints.values()

        js.actual.velocities = [0.0] * len(joints)
        js.actual.accelerations = [0.0] * len(joints)

        self.publish(js)
