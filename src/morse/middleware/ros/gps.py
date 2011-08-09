import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('nav_msgs'); roslib.load_manifest('rosgraph_msgs'); roslib.load_manifest('geometry_msgs')
import rospy
import std_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import GameLogic
import math
import mathutils

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
    if mw_data[1] == "post_odometry":  
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, Odometry))
    else:
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, PoseStamped)) 
    
    logger.info('######## ROS POSE PUBLISHER INITIALIZED ########')
    
def post_odometry(self, component_instance):
    """ Publish the data of the Pose-sensor as a ROS-Odometry message.

    """
    parent_name = component_instance.robot_parent.blender_obj.name
    odometry = Odometry()
        
    odometry.pose.pose.position.x = component_instance.local_data['x']
    odometry.pose.pose.position.y = component_instance.local_data['y']
    odometry.pose.pose.position.z = component_instance.local_data['z']
    
    odometry.pose.pose.orientation.w = 0
    odometry.pose.pose.orientation.x = 0
    odometry.pose.pose.orientation.y = 0
    odometry.pose.pose.orientation.z = 0
    
    odometry.header.stamp = rospy.Time.now()
    # Default baseframe is map  
    odometry.header.frame_id = "map"
    
    for topic in self._topics: 
        message = odometry
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(odometry)

def post_poseStamped(self, component_instance):
    """ Publish the data of the Pose as a ROS-PoseStamped message
    """
    parent_name = component_instance.robot_parent.blender_obj.name
    poseStamped = PoseStamped()
        
    poseStamped.pose.position.x = component_instance.local_data['x']
    poseStamped.pose.position.y = component_instance.local_data['y']
    poseStamped.pose.position.z = component_instance.local_data['z']
    
    poseStamped.pose.orientation.w = 0
    poseStamped.pose.orientation.x = 0
    poseStamped.pose.orientation.y = 0
    poseStamped.pose.orientation.z = 0
    
    poseStamped.header.stamp = rospy.Time.now()
    # Default baseframe is map  
    poseStamped.header.frame_id = "map"
    
    for topic in self._topics: 
        message = poseStamped
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name): 
            topic.publish(poseStamped)
