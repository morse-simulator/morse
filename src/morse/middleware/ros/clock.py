import bge
import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('rosgraph_msgs');
import rospy
from rosgraph_msgs.msg import Clock

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self._sim_time = 0.0
    # LogicTicRate default value: 60 Hz
    self._sim_step = 1.0 / bge.logic.getLogicTicRate()

    self.register_publisher(component_instance, function, Clock)

def post_clock(self, component_instance):
    """ Publish the simulator clock as a Clock ROS msg

    """
    msg = Clock()
    msg.clock = rospy.Time.from_sec(self._sim_time)
    self._sim_time += self._sim_step

    self.publish(msg, component_instance)
