import roslib; roslib.load_manifest('std_msgs')
from std_msgs.msg import Float32

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_publisher(component_instance, function, Float32)

def post_float32(self, component_instance):
    """ Publish the data of the battery sensor as a single float32 message.

    """
    msg = Float32()
    msg.data = component_instance.local_data['charge']

    self.publish(msg, component_instance)
