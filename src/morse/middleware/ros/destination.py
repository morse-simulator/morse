import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Point

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_subscriber(component_instance, function, Point, callback_wp)

def callback_wp(data, component_instance):
    """ this function is called as soon as Point messages are published on the specific topic """
    component_instance.local_data["x"] = data.x
    component_instance.local_data["y"] = data.y
    component_instance.local_data["z"] = data.z

def read_point(self, component_instance):
    """ dummy function for Waypoints """
