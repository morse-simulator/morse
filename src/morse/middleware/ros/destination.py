import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs');
import rospy
from geometry_msgs.msg import Point

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name

    # Add the new method to the component
    component_instance.input_functions.append(function)
    self._topics.append(rospy.Subscriber(parent_name + "/" + component_name, Point, callback_wp, component_instance))

def callback_wp(data, component_instance):
    """ this function is called as soon as Point messages are published on the specific topic """
    component_instance.local_data["x"] = data.x
    component_instance.local_data["y"] = data.y
    component_instance.local_data["z"] = data.z

def read_point(self, component_instance):
    """ dummy function for Waypoints """
