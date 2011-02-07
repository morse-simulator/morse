import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('nav_msgs'); roslib.load_manifest('rosgraph_msgs')  
import rospy
import std_msgs
from nav_msgs.msg import Odometry

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
    self._topics.append(rospy.Publisher(parent_name + "/" + component_name, Odometry))    

#Note: posting 2D Laserscans is still experimental and maybe the ranges are not correctly calculated!
def post_odometry(self, component_instance):
    """ Publish the data on the rostopic
	"""
    parent_name = component_instance.robot_parent.blender_obj.name
    odometry = Odometry()
        
    odometry.pose.pose.position.x = component_instance.position_3d.x
    odometry.pose.pose.position.y = component_instance.position_3d.y
    odometry.pose.pose.position.z = component_instance.position_3d.z
    odometry.pose.pose.orientation.x = component_instance.position_3d.roll
    odometry.pose.pose.orientation.x = component_instance.position_3d.pitch
    odometry.pose.pose.orientation.x = component_instance.position_3d.yaw
        
    for topic in self._topics: 
        message = odometry
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(odometry)
