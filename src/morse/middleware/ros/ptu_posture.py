import roslib; roslib.load_manifest('sensor_msgs')
from sensor_msgs.msg import JointState

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_publisher(component_instance, function, JointState)

def post_jointState(self, component_instance):
    """ Publish the data of the Odometry-sensor as a ROS-Pose message
    """
    js = JointState()
    js.header = self.get_ros_header(component_instance)
    js.name = ['head_pan_joint', 'head_tilt_joint']

    js.position = [
        component_instance.local_data['pan'],
        component_instance.local_data['tilt']
    ]

    self.publish(js, component_instance)
