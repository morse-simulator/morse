import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs'); roslib.load_manifest('rosgraph_msgs')  
import rospy
import std_msgs
import math
from geometry_msgs.msg import Wrench

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name
    
    # Add the new method to the component
    component_instance.input_functions.append(function)
    self._topics.append(rospy.Subscriber(parent_name + "/" + component_name, Wrench, callback_wp, component_instance))

def callback_wp(data, component_instance):
    """ this function is called as soon as Wrench messages are published on the specific topic """
    component_instance.local_data["force"][0] = data.force.x
    component_instance.local_data["force"][1] = data.force.y
    component_instance.local_data["force"][2] = data.force.z
    component_instance.local_data["torque"][0] = data.torque.x
    component_instance.local_data["torque"][1] = data.torque.y
    component_instance.local_data["torque"][2] = data.torque.z
    logger.debug("Applying force: [%s, %s, %s], torque: [%s, %s, %s]"%(data.force.x, data.force.y, data.force.z, data.torque.x, data.torque.y, data.torque.z))
        
def read_wrench(self, component_instance):
    """ dummy function force_torque controller """
