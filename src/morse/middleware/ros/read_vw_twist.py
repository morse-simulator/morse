import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Twist

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_subscriber(component_instance, function, Twist, callback_twist)

def callback_twist(data, component_instance):
    """ this function is called as soon as Twist messages are published on the specific topic """
    component_instance.local_data["v"] = data.linear.x
    component_instance.local_data["w"] = data.angular.z # yaw

def read_twist(self, component_instance):
    """ dummy function for Waypoints """
