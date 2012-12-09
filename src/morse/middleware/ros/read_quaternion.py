import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('geometry_msgs')
import math
import mathutils
from geometry_msgs.msg import Quaternion

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_subscriber(component_instance, function, Quaternion, callback_quaternion)

def callback_quaternion(data, component_instance):
    """ this function is called as soon as Quaternion messages are published on the specific topic """
    quat = mathutils.Quaternion((data.w, data.x, data.y, data.z))
    euler = quat.to_euler('XYZ')
    component_instance.local_data["roll"] = euler.x
    component_instance.local_data["pitch"] = euler.y
    component_instance.local_data["yaw"] = euler.z
    logger.debug("Set orientation to RPY (%.3f %.3f %.3f)" % tuple(math.degrees(a) for a in euler))

def read_quaternion(self, component_instance):
    """ dummy function orientation controller """
