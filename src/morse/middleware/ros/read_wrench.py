import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('geometry_msgs')
from geometry_msgs.msg import Wrench

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_subscriber(component_instance, function, Wrench, callback_wrench)

def callback_wrench(data, component_instance):
    """ this function is called as soon as Wrench messages are published on the specific topic """
    component_instance.local_data["force"][0] = data.force.x
    component_instance.local_data["force"][1] = data.force.y
    component_instance.local_data["force"][2] = data.force.z
    component_instance.local_data["torque"][0] = data.torque.x
    component_instance.local_data["torque"][1] = data.torque.y
    component_instance.local_data["torque"][2] = data.torque.z
    logger.debug("Applying force: [%s, %s, %s], torque: [%s, %s, %s]"%
                 (data.force.x, data.force.y, data.force.z,
                  data.torque.x, data.torque.y, data.torque.z))

def read_wrench(self, component_instance):
    """ dummy function force_torque controller """
