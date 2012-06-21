import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs'); roslib.load_manifest('rosgraph_msgs')
import rospy
import std_msgs
import math
import mathutils
from geometry_msgs.msg import Pose

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name

    # Add the new method to the component
    component_instance.input_functions.append(function)
    self._topics.append(rospy.Subscriber(parent_name + "/" + component_name, Pose, callback_pose, component_instance))

def callback_pose(data, component_instance):
    """ this function is called as soon as Pose messages are published on the specific topic """
    pos = mathutils.Vector((data.position.x, data.position.y, data.position.z))
    component_instance.local_data["x"] = pos.x
    component_instance.local_data["y"] = pos.y
    component_instance.local_data["z"] = pos.z
    logger.debug("Set position to (%.3f %.3f %.3f)" % (pos.x, pos.y, pos.z))

    quat = mathutils.Quaternion((data.orientation.w, data.orientation.x, data.orientation.y, data.orientation.z))
    component_instance.local_data['qw'] = quat.w
    component_instance.local_data['qx'] = quat.x
    component_instance.local_data['qy'] = quat.y
    component_instance.local_data['qz'] = quat.z
    euler = quat.to_euler()
    logger.debug("Set orientation to RPY (%.3f %.3f %.3f)" % (euler.x * 180. / math.pi, euler.y * 180. / math.pi, euler.z * 180. / math.pi))

def read_pose(self, component_instance):
    """ dummy function orientation controller """
