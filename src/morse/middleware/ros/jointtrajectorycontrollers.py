from morse.middleware.sockets.jointstate import fill_missing_pr2_joints

# ROS imports
from pr2_controllers_msgs.msg import JointTrajectoryControllerState

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_instance.output_functions.append(function)
    topic = mw_data[3].get("topic", self.get_topic_name(component_instance))
    self.set_topic_name(component_instance, topic)
    self.register_publisher_name_class(topic, JointTrajectoryControllerState)

def post_controller_state(self, component_instance):

    data = component_instance.local_data

    js = JointTrajectoryControllerState()
    js.header = self.get_ros_header(component_instance)

    js.joint_names = data.keys()
    js.actual.positions = data.values()

    js.actual.velocities = [0.0] * len(data)
    js.actual.accelerations = [0.0] * len(data)

    self.publish(js, component_instance)
