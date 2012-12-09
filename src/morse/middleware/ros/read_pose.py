import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('geometry_msgs')
import math
import mathutils
from geometry_msgs.msg import Pose

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_subscriber(component_instance, function, Pose, callback_pose)

def callback_pose(data, component_instance):
    """ this function is called as soon as Pose messages are published on the specific topic """
    pos = mathutils.Vector((data.position.x, data.position.y, data.position.z))
    component_instance.local_data["x"] = pos.x
    component_instance.local_data["y"] = pos.y
    component_instance.local_data["z"] = pos.z
    #logger.debug("Set position to (%.3f %.3f %.3f)" % (pos.x, pos.y, pos.z))

    quat = mathutils.Quaternion((data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z))
    euler = quat.to_euler()
    component_instance.local_data['roll'] = euler.x
    component_instance.local_data['pitch'] = euler.y
    component_instance.local_data['yaw'] = euler.z
    #logger.debug("Set orientation to RPY (%.3f %.3f %.3f)" % tuple(math.degrees(a) for a in euler))

def read_pose(self, component_instance):
    """ dummy function orientation controller """
