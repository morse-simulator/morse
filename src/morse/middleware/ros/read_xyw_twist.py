import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs'); roslib.load_manifest('rosgraph_msgs')  
import rospy
import std_msgs
import math
from geometry_msgs.msg import Twist

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name
    
    # Add the new method to the component
    component_instance.input_functions.append(function)
    self._topics.append(rospy.Subscriber(parent_name + "/" + component_name, Twist, callback_wp, component_instance))

def callback_wp(data, component_instance):
    """ this function is called as soon as Twist messages are published on the specific topic """
    component_instance.local_data["x"] = data.linear.x
    component_instance.local_data["y"] = data.linear.y

    yaw = data.angular.z
    component_instance.local_data["w"] = yaw 
    logger.debug("Executing v-omega-movement: linear: < %s, %s>"%(data.linear.x, yaw))
        
def read_twist(self, component_instance):
    """ dummy function for Waypoints """
