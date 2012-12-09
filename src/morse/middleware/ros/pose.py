import logging; logger = logging.getLogger("morse." + __name__)
import mathutils
import roslib; roslib.load_manifest('geometry_msgs'); roslib.load_manifest('nav_msgs')
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from morse.middleware.ros.tfMessage import tfMessage

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # default frame_ids
    frame_id = "/map"
    child_frame_id = "/base_footprint"
    # Extract the Middleware parameters. additional parameter should be a dict
    try:
        frame_id = mw_data[3].get("frame_id", "/map")
        child_frame_id = mw_data[3].get("child_frame_id", "/base_footprint")
    except:
        pass

    self.set_property(component_instance, 'frame_id', frame_id)
    self.set_property(component_instance, 'child_frame_id', child_frame_id)

    if mw_data[1] == "post_tf":
        self.register_publisher_tf()
        component_instance.output_functions.append(function)
    else:
        self.register_publisher(component_instance, function, get_ros_class(mw_data[1]))

    logger.info("Initialized the ROS Pose sensor with frame_id '%s' and child_frame_id '%s'",
                frame_id, child_frame_id)

def get_ros_class(method_name):
    dict_method_class = {
        'post_odometry': Odometry,
        'post_pose': PoseStamped,
    }
    return dict_method_class[method_name]

def post_tf(self, component_instance):
    t = TransformStamped()
    t.header = self.get_ros_header(component_instance)
    t.header.frame_id = self.get_property(component_instance, 'frame_id')
    t.child_frame_id = self.get_property(component_instance, 'child_frame_id')

    t.transform.translation.x = component_instance.local_data['x']
    t.transform.translation.y = component_instance.local_data['y']
    t.transform.translation.z = component_instance.local_data['z']

    euler = mathutils.Euler((component_instance.local_data['roll'],
                             component_instance.local_data['pitch'],
                             component_instance.local_data['yaw']))
    t.transform.rotation = euler.to_quaternion()

    tfm = tfMessage([t])

    self.publish_topic(tfm, "/tf")

def post_odometry(self, component_instance):
    """ Publish the data of the Pose as a Odometry message for fake localization
    """
    odometry = Odometry()
    odometry.header = self.get_ros_header(component_instance)
    odometry.header.frame_id = self.get_property(component_instance, 'frame_id')
    odometry.child_frame_id = self.get_property(component_instance, 'child_frame_id')

    odometry.pose.pose.position.x = component_instance.local_data['x']
    odometry.pose.pose.position.y = component_instance.local_data['y']
    odometry.pose.pose.position.z = component_instance.local_data['z']

    euler = mathutils.Euler((component_instance.local_data['roll'],
                             component_instance.local_data['pitch'],
                             component_instance.local_data['yaw']))
    odometry.pose.pose.orientation = euler.to_quaternion()

    self.publish(odometry, component_instance)

def post_pose(self, component_instance):
    """ Publish the data of the Pose as a ROS PoseStamped message
    """
    pose = PoseStamped()
    pose.header = self.get_ros_header(component_instance)
    pose.header.frame_id = self.get_property(component_instance, 'frame_id')

    pose.pose.position.x = component_instance.local_data['x']
    pose.pose.position.y = component_instance.local_data['y']
    pose.pose.position.z = component_instance.local_data['z']

    euler = mathutils.Euler((component_instance.local_data['roll'],
                             component_instance.local_data['pitch'],
                             component_instance.local_data['yaw']))
    pose.pose.orientation = euler.to_quaternion()

    self.publish(pose, component_instance)
