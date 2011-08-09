import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('nav_msgs'); roslib.load_manifest('rosgraph_msgs'); roslib.load_manifest('geometry_msgs')#; roslib.load_manifest('tf')
import rospy
import std_msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
#from tf import TransformBroadcaster
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
    if mw_data[1] == "post_odometry":  
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, Odometry))
    elif mw_data[1] == "post_odometry_transform":
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, Odometry))
        self._previous_orientation = [0.0, 0.0, 0.0]
        self._previous_position = [0.0, 0.0, 0.0]
        self._previous_time = 0.0
        #self.odom_broadcaster = TransformBroadcaster()
        # TF has not been patched Python3 yet... (TODO)
        self._seq = 0
    else:
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, PoseStamped)) 

    logger.info('######## ROS POSE PUBLISHER INITIALIZED ########')

def post_odometry_transform(self, component_instance):
    """ Publish the data of the Pose-sensor as a ROS-Odometry message.

    cf. http://www.ros.org/doc/api/nav_msgs/html/msg/Odometry.html , 
    http://www.ros.org/wiki/navigation/Tutorials/RobotSetup/Odom#Writing_the_Code
    """
    parent_name = component_instance.robot_parent.blender_obj.name
    current_time = rospy.Time.now()

    x = component_instance.local_data['x']
    y = component_instance.local_data['y']
    z = component_instance.local_data['z']
    roll = component_instance.local_data['roll']
    pitch = component_instance.local_data['pitch']
    yaw = component_instance.local_data['yaw']

    euler = mathutils.Euler((roll, pitch, yaw))
    # temporarily support deprecated Python3.1/Blender2.56
    if sys.version_info.minor == 1:
        quaternion = euler.to_quat()
    else:
        quaternion = euler.to_quaternion()

    #odom_trans = TransformStamped()
    #odom_trans.header.stamp = current_time
    #odom_trans.header.frame_id = "odom"
    #odom_trans.child_frame_id = "base_link"
    # transform tree data
    #odom_trans.transform.translation.x = x
    #odom_trans.transform.translation.y = y
    #odom_trans.transform.translation.z = z
    #odom_trans.transform.rotation = quaternion
    #self.odom_broadcaster.sendTransform(odom_trans)
    # send the transform
    #odom_broadcaster.sendTransform(odom_trans)

    odometry = Odometry()
    odometry.header.seq = self._seq
    odometry.header.stamp = current_time
    odometry.header.frame_id = "/base_link"
    odometry.child_frame_id = "/odom"

    odometry.pose.pose.position.x = x
    odometry.pose.pose.position.y = y
    odometry.pose.pose.position.z = z
    odometry.pose.pose.orientation = quaternion

    # compute twist
    current_sec = current_time.to_sec()
    dt = current_sec - self._previous_time
    odometry.twist.twist.linear.x = dt * (x - self._previous_position[0])
    odometry.twist.twist.linear.y = dt * (y - self._previous_position[1])
    odometry.twist.twist.linear.z = dt * (z - self._previous_position[2])
    odometry.twist.twist.angular.x = dt * (roll - self._previous_orientation[0])
    odometry.twist.twist.angular.y = dt * (pitch - self._previous_orientation[1])
    odometry.twist.twist.angular.z = dt * (yaw - self._previous_orientation[2])
    # store previous data
    self._previous_orientation = [roll, pitch, yaw]
    self._previous_position = [x, y, z]
    self._previous_time = current_sec

    for topic in self._topics:
        # publish the message on the correct topic
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(odometry)

    self._seq = self._seq + 1

def post_odometry(self, component_instance):
    """ Publish the data of the Pose-sensor as a ROS-Odometry message.

    """
    parent_name = component_instance.robot_parent.blender_obj.name
    odometry = Odometry()
        
    odometry.pose.pose.position.x = component_instance.local_data['x']
    odometry.pose.pose.position.y = component_instance.local_data['y']
    odometry.pose.pose.position.z = component_instance.local_data['z']
    euler = mathutils.Euler((component_instance.local_data['roll'], component_instance.local_data['pitch'], component_instance.local_data['yaw']))
    
    # temporarily support deprecated Python3.1/Blender2.56
    if sys.version_info.minor == 1:
        quaternion = euler.to_quat()
    else:
        quaternion = euler.to_quaternion()

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
    # temporarily support deprecated Python3.1/Blender2.56
    if sys.version_info.minor == 1:
        quaternion = euler.to_quat()
    else:
        quaternion = euler.to_quaternion()

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
