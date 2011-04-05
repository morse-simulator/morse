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
    
    print ('######## ROS POSE PUBLISHER INITIALIZED ########')
    
def post_odometry(self, component_instance):
    """ Publish the data of the Pose-sensor as a ROS-Odometry message.

    """
    parent_name = component_instance.robot_parent.blender_obj.name
    odometry = Odometry()
        
    odometry.pose.pose.position.x = component_instance.local_data['x']
    odometry.pose.pose.position.y = component_instance.local_data['y']
    odometry.pose.pose.position.z = component_instance.local_data['z']
    euler = mathutils.Euler((component_instance.local_data['roll'], component_instance.local_data['pitch'], component_instance.local_data['yaw']))
    quaternion = euler.to_quat()
    odometry.pose.pose.orientation.w = quaternion.w
    odometry.pose.pose.orientation.x = quaternion.x
    odometry.pose.pose.orientation.y = quaternion.y
    odometry.pose.pose.orientation.z = quaternion.z
    
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
    euler = mathutils.Euler((component_instance.local_data['roll'], component_instance.local_data['pitch'], component_instance.local_data['yaw']))
    quaternion = euler.to_quat()
    poseStamped.pose.orientation.w = quaternion.w
    poseStamped.pose.orientation.x = quaternion.x
    poseStamped.pose.orientation.y = quaternion.y
    poseStamped.pose.orientation.z = quaternion.z
    
    poseStamped.header.stamp = rospy.Time.now()
    # Default baseframe is map  
    poseStamped.header.frame_id = "map"
    
    for topic in self._topics: 
        message = poseStamped
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name): 
            topic.publish(poseStamped)
