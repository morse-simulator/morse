import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs'); roslib.load_manifest('rosgraph_msgs')  
import rospy
import std_msgs
import math
from geometry_msgs.msg import Twist

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Add the new method to the component
    component_instance.input_functions.append(function)
    self._topics.append(rospy.Subscriber("/cmd_vel", Twist, callback_wp, component_instance))

def callback_wp(data, component_instance):
        """ this function is called as soon as Twist messages are published on the specific topic """
        component_instance.local_data["v"] = data.linear.x
        yaw = data.angular.z
        component_instance.local_data["w"] = yaw 
        #print("Executing v-omega-movement: linear: < %s, %s>"%(data.linear.x, yaw))
        
def read_twist(self, component_instance):
        """ dummy function for Waypoints """
