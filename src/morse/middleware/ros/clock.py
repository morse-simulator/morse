import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('rosgraph_msgs');
import rospy
from rosgraph_msgs.msg import Clock
from morse.middleware.ros import ROSPublisher

from morse.core import blenderapi

class ClockPublisher(ROSPublisher):
    """ Publish the simulator clock. """
    ros_class = Clock

    def initialize(self):
        ROSPublisher.initialize(self)
        self._sim_time = 0.0
        # LogicTicRate default value: 60 Hz
        self._sim_step = 1.0 / blenderapi.getfrequency()

    def default(self, ci='unused'):
        msg = Clock()
        msg.clock = rospy.Time.from_sec(self._sim_time)
        self._sim_time += self._sim_step

        self.publish(msg)
