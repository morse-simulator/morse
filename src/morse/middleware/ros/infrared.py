import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('sensor_msgs')
import rospy
from sensor_msgs.msg import Range

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_publisher(component_instance, function, Range)

def post_range(self, component_instance):
    """ Publish the data on the rostopic

    http://www.ros.org/doc/api/sensor_msgs/html/msg/Range.html
    """
    msg = Range()
    #msg.header.frame_id = 'infrared'
    msg.radiation_type = Range.INFRARED
    msg.field_of_view = 20
    msg.min_range = 0
    msg.max_range = component_instance.blender_obj['laser_range']
    tmp = component_instance.blender_obj['laser_range']
    for r in component_instance.local_data['range_list']:
        if tmp > r:
            tmp = r
    msg.range = tmp

    self.publish(msg, component_instance)
