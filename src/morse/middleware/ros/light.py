import roslib; roslib.load_manifest('std_msgs')
from std_msgs.msg import Bool

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_subscriber(component_instance, function, Bool, callback_light)

def callback_light(data, component_instance):
    """ this function is called as soon as Twist messages are published on the specific topic """
    component_instance.local_data["emit"] = data.data

def read_switch(self, component_instance):
    """ dummy function for Light """
