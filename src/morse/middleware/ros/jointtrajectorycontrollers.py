# ROS imports: pr2_controllers_msgs is not catkinized in fuerte
from morse.middleware.ros.helpers import ros_add_to_syspath
ros_add_to_syspath("pr2_controllers_msgs")
from pr2_controllers_msgs.msg import JointTrajectoryControllerState
from morse.middleware.ros import ROSPublisher

class JointTrajectoryControllerStatePublisher(ROSPublisher):
    """ Publish the data of the pr2 joint sensor. """
    ros_class = JointTrajectoryControllerState

    def default(self, ci='unused'):
        js = JointTrajectoryControllerState()
        js.header = self.get_ros_header()

        js.joint_names = self.data.keys()
        js.actual.positions = self.data.values()

        js.actual.velocities = [0.0] * len(self.data)
        js.actual.accelerations = [0.0] * len(self.data)

        self.publish(js)
