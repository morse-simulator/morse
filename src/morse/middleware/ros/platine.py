import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs')
import rospy
import std_msgs
import math
from geometry_msgs.msg import Vector3

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    This module reads a Vector3-ROS message and sets the local data for "pan" and "tilt" according to the rotation axis (pan: y-axis, tilt: z-axis)
    """
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name
    
    # Add the new method to the component
    component_instance.input_functions.append(function)
    self._topics.append(rospy.Subscriber(parent_name + "/" + component_name, Vector3, callback_wp, component_instance))

def callback_wp(data, component_instance):
    """ this function is called as soon as Vector3-messages are published on the specific topic """
    component_instance.local_data["pan"] = data.y
    component_instance.local_data["tilt"] = data.z
        
def read_Vector3(self, component_instance):
    """ dummy function for Platine controller """
