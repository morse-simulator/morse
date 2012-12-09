import roslib; roslib.load_manifest('nav_msgs'); roslib.load_manifest('geometry_msgs')
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_publisher(component_instance, function, get_ros_class(mw_data[1]))

def get_ros_class(method_name):
    dict_method_class = {
        'post_odometry': Odometry,
        'post_poseStamped': PoseStamped,
    }
    return dict_method_class[method_name]

def post_odometry(self, component_instance):
    """ Publish the data of the Pose-sensor as a ROS-Odometry message.

    """
    odometry = Odometry()
    odometry.header = self.get_ros_header(component_instance)
    # Default baseframe is map
    odometry.header.frame_id = "map"

    odometry.pose.pose.position.x = component_instance.local_data['x']
    odometry.pose.pose.position.y = component_instance.local_data['y']
    odometry.pose.pose.position.z = component_instance.local_data['z']

    odometry.pose.pose.orientation.w = 0
    odometry.pose.pose.orientation.x = 0
    odometry.pose.pose.orientation.y = 0
    odometry.pose.pose.orientation.z = 0

    self.publish(odometry, component_instance)

def post_poseStamped(self, component_instance):
    """ Publish the data of the Pose as a ROS-PoseStamped message
    """
    pose = PoseStamped()
    pose.header = self.get_ros_header(component_instance)
    # Default baseframe is map
    pose.header.frame_id = "map"

    pose.pose.position.x = component_instance.local_data['x']
    pose.pose.position.y = component_instance.local_data['y']
    pose.pose.position.z = component_instance.local_data['z']

    pose.pose.orientation.w = 0
    pose.pose.orientation.x = 0
    pose.pose.orientation.y = 0
    pose.pose.orientation.z = 0

    self.publish(pose, component_instance)
