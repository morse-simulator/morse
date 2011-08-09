import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs')
import rospy
import std_msgs
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import GameLogic
import math
import mathutils
import sys

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
    else:
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, Twist)) 
    
    logger.info('######## ROS POSE PUBLISHER INITIALIZED ########')

def post_pose(self, component_instance):
    """ Publish the data of the Odometry-sensor as a ROS-Pose message
    """
    parent_name = component_instance.robot_parent.blender_obj.name
    pose = PoseStamped()
        
    pose.pose.position.x = component_instance.local_data['dx']
    pose.pose.position.y = component_instance.local_data['dy']
    pose.pose.position.z = component_instance.local_data['dz']
    euler = mathutils.Euler((component_instance.local_data['droll'], component_instance.local_data['dpitch'], component_instance.local_data['dyaw']))
    
    # temporarily support deprecated Python3.1/Blender2.56
    if sys.version_info.minor == 1:
        quaternion = euler.to_quat()
    else:
        quaternion = euler.to_quaternion()

    pose.pose.orientation.w = quaternion.w
    pose.pose.orientation.x = quaternion.x
    pose.pose.orientation.y = quaternion.y
    pose.pose.orientation.z = quaternion.z
    
    pose.header.stamp = rospy.Time.now()
    pose.header.frame_id = "/odom"

    for topic in self._topics: 
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name): 
            topic.publish(pose)

def post_twist(self, component_instance):
    """ Publish the data of the Odometry sensor as a ROS-Twist message
    """
    parent_name = component_instance.robot_parent.blender_obj.name
    twist = Twist()
        
    twist.linear.x = component_instance.local_data['dx']
    twist.linear.y = component_instance.local_data['dy']
    twist.linear.z = component_instance.local_data['dz']
    twist.angular.x = component_instance.local_data['droll']
    twist.angular.y = component_instance.local_data['dpitch']
    twist.angular.z = component_instance.local_data['dyaw']
          
    for topic in self._topics: 
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name): 
            topic.publish(twist)
