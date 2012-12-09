import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Vector3

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    This module reads a Vector3-ROS message and sets the local data
    for "pan" and "tilt" according to the rotation axis (pan: y-axis, tilt: z-axis)
    """
    self.register_subscriber(component_instance, function, Vector3, callback_wp)

def callback_wp(data, component_instance):
    """ called as soon as Vector3 messages are published on the specific topic """
    component_instance.local_data["pan"] = data.y
    component_instance.local_data["tilt"] = data.z

def read_Vector3(self, component_instance):
    """ dummy function for Platine controller """
