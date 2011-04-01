import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs'); roslib.load_manifest('rosgraph_msgs')
import rospy
import std_msgs
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
    self._topics.append(rospy.Publisher(parent_name + "/" + component_name, PoseStamped))    
    print ('######## POSESTAMPED-MSG PUBLISHER INITIALIZED ########')
    
def post_poseStamped(self, component_instance):
    """ Publish the data of any Blender object as a PoseStamped-ROS message.

    This extended GPS sensor can thus be used to do a fake-localization 
    """
    parent_name = component_instance.robot_parent.blender_obj.name
    odometry = PoseStamped()
    odometry.pose.position.x = component_instance.position_3d.x
    odometry.pose.position.y = component_instance.position_3d.y
    odometry.pose.position.z = component_instance.position_3d.z
    euler = mathutils.Euler((component_instance.position_3d.roll, component_instance.position_3d.pitch, component_instance.position_3d.yaw))
    quaternion = euler.to_quat()
    odometry.pose.orientation.w = quaternion.w
    odometry.pose.orientation.x = quaternion.x
    odometry.pose.orientation.y = quaternion.y
    odometry.pose.orientation.z = quaternion.z
    
    odometry.header.stamp = rospy.Time.now()
    # For truepose, baseframe is map
    odometry.header.frame_id = "map"
    
    for topic in self._topics: 
        message = odometry
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(odometry)
