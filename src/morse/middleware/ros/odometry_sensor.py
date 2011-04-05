import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs')
import rospy
import std_msgs
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist
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
    if mw_data[1] == "post_pose":  
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, Pose))
    else:
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, Twist)) 
    
    print('######## ROS POSE PUBLISHER INITIALIZED ########')

def post_pose(self, component_instance):
    """ Publish the data of the Odometry-sensor as a ROS-Pose message
    """
    parent_name = component_instance.robot_parent.blender_obj.name
    pose = Pose()
        
    pose.position.x = component_instance.local_data['dx']
    pose.position.y = component_instance.local_data['dy']
    pose.position.z = component_instance.local_data['dz']
    euler = mathutils.Euler((component_instance.local_data['droll'], component_instance.local_data['dpitch'], component_instance.local_data['dyaw']))
    quaternion = euler.to_quat()
    pose.orientation.w = quaternion.w
    pose.orientation.x = quaternion.x
    pose.orientation.y = quaternion.y
    pose.orientation.z = quaternion.z
          
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
