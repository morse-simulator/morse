import bge
import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('rosgraph_msgs');
import rospy
from rosgraph_msgs.msg import Clock

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.blender_obj.name

    # Add the new method to the component
    component_instance.output_functions.append(function)

    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    self._topics.append(rospy.Publisher(component_name, Clock))
    self._sim_time = 0.0
    self._sim_step = 1.0 / bge.logic.getLogicTicRate()
    # LogicTicRate default value: 60 Hz

    logger.info('ROS publisher initialized')

def post_clock(self, component_instance):
    """ Publish the simulator clock as a Clock ROS msg

    """
    msg = Clock()
    msg.clock = rospy.Time.from_sec(self._sim_time)
    self._sim_time += self._sim_step

    for topic in self._topics:
        # publish the message on the correct topic
        if str(topic.name) == str("/" + component_instance.blender_obj.name):
            topic.publish(msg)

