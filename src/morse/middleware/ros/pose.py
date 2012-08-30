import logging; logger = logging.getLogger("morse." + __name__)

import bge
import math
import mathutils

import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs'); roslib.load_manifest('nav_msgs')

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from morse.middleware.ros.tfMessage import tfMessage

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name

    # Add the new method to the component
    component_instance.output_functions.append(function)

    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    if mw_data[1] == "post_pose":
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, PoseStamped))
    elif mw_data[1] == "post_tf":
        self._topics.append(rospy.Publisher("/tf", tfMessage))
    else:
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, Odometry))

    # Extract the Middleware parameters
    # additional parameter should be a dict
    try:
        frame_id = mw_data[3].get("frame_id", "/map")
        child_frame_id = mw_data[3].get("child_frame_id", "/base_footprint")
    except:
        frame_id = "/map"
        child_frame_id = "/base_footprint"

    # create a new dictionary for this sensor if necessary
    if component_name not in self._properties:
        self._properties[component_name] = {}
    # store the frame ids in the dict
    self._properties[component_name]['frame_id'] = frame_id
    self._properties[component_name]['child_frame_id'] = child_frame_id

    logger.info("Initialized the ROS pose sensor with frame_id '%s' and child_frame_id '%s'",
                frame_id, child_frame_id)

def post_tf(self, component_instance):
    component_name = component_instance.blender_obj.name
    frame_id = self._properties[component_name]['frame_id']
    child_frame_id = self._properties[component_name]['child_frame_id']

    euler = mathutils.Euler((component_instance.local_data['roll'],
                             component_instance.local_data['pitch'],
                             component_instance.local_data['yaw']))
    quaternion = euler.to_quaternion()

    t = TransformStamped()
    t.header.frame_id = frame_id
    t.header.stamp = rospy.Time.now()
    t.child_frame_id = child_frame_id
    t.transform.translation.x = component_instance.local_data['x']
    t.transform.translation.y = component_instance.local_data['y']
    t.transform.translation.z = component_instance.local_data['z']

    t.transform.rotation = quaternion

    tfm = tfMessage([t])

    for topic in self._topics:
        # publish the message on the correct topic    
        if str(topic.name) == str("/tf"):
            topic.publish(tfm)


def post_odometry(self, component_instance):
    """ Publish the data of the Pose as a Odometry message for fake localization 
    """
    parent_name = component_instance.robot_parent.blender_obj.name
    component_name = component_instance.blender_obj.name
    frame_id = self._properties[component_name]['frame_id']
    child_frame_id = self._properties[component_name]['child_frame_id']

    odometry = Odometry()
    odometry.header.stamp = rospy.Time.now()
    odometry.header.frame_id = frame_id
    odometry.child_frame_id = child_frame_id

    odometry.pose.pose.position.x = component_instance.local_data['x']
    odometry.pose.pose.position.y = component_instance.local_data['y']
    odometry.pose.pose.position.z = component_instance.local_data['z']

    euler = mathutils.Euler((component_instance.local_data['roll'],
                             component_instance.local_data['pitch'],
                             component_instance.local_data['yaw']))
    quaternion = euler.to_quaternion()
    odometry.pose.pose.orientation = quaternion

    for topic in self._topics:
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_name):
            topic.publish(odometry)

def post_pose(self, component_instance):
    """ Publish the data of the Pose as a ROS-PoseStamped message
    """
    parent_name = component_instance.robot_parent.blender_obj.name
    component_name = component_instance.blender_obj.name
    frame_id = self._properties[component_name]['frame_id']

    poseStamped = PoseStamped()

    poseStamped.pose.position.x = component_instance.local_data['x']
    poseStamped.pose.position.y = component_instance.local_data['y']
    poseStamped.pose.position.z = component_instance.local_data['z']
    euler = mathutils.Euler((component_instance.local_data['roll'],
                             component_instance.local_data['pitch'],
                             component_instance.local_data['yaw']))
    quaternion = euler.to_quaternion()
    poseStamped.pose.orientation.w = quaternion.w
    poseStamped.pose.orientation.x = quaternion.x
    poseStamped.pose.orientation.y = quaternion.y
    poseStamped.pose.orientation.z = quaternion.z

    poseStamped.header.stamp = rospy.Time.now()

    # Default baseframe is map  
    poseStamped.header.frame_id = frame_id

    for topic in self._topics:
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_name):
            topic.publish(poseStamped)
