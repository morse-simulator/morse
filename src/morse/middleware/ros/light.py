import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('std_msgs')
import rospy
from std_msgs.msg import Bool

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name
    
    # Add the new method to the component
    component_instance.input_functions.append(function)

    # Generate one subscriber and one topic for each component that is a sensor and uses post_message
    self._topics.append(rospy.Subscriber(parent_name + "/" + component_name, Bool, callback_wp, component_instance))

    logger.info('ROS subscriber initialized')

def callback_wp(data, component_instance):
        """ this function is called as soon as Twist messages are published on the specific topic """
        component_instance.local_data["emit"] = data.data

def read_switch(self, component_instance):
        """ dummy function for Waypoints """

