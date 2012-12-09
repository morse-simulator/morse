import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Twist

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_publisher(component_instance, function, Twist)

def post_twist(self, component_instance):
    """ Publish the data of the acceleromter sensor as a ROS Twist message

    Only the velocity part is exported.
    """
    twist = Twist()

    # Fill twist-msg with the values from the sensor
    twist.linear.x = component_instance.local_data['velocity'][0]
    twist.linear.y = component_instance.local_data['velocity'][1]
    twist.linear.z = component_instance.local_data['velocity'][2]

    self.publish(twist, component_instance)
