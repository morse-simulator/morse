import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('sensor_msgs')
import rospy
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2

from morse.middleware.ros import point_cloud2

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    # Compose the name of the port, based on the parent and module names
    component_name = component_instance.blender_obj.name
    parent_name = component_instance.robot_parent.blender_obj.name

    # Add the new method to the component
    component_instance.output_functions.append(function)

    # Generate one publisher and one topic for each component that is a sensor and uses post_message
    self._topics.append(rospy.Publisher('/' + parent_name + '/' + component_name, PointCloud2))
    self._seq = 0

    logger.info('ROS publisher for depth camera initialized')

def post_pointcloud2(self, component_instance):
    """ Publish the data of the Depth Camera as a ROS-PointCloud2 message.

    """
    if not component_instance.capturing:
        return # press [Space] key to enable capturing

    parent_name = component_instance.robot_parent.blender_obj.name
    component_name = component_instance.blender_obj.name

    header = Header()
    header.stamp = rospy.Time.now()
    header.seq = self._seq
    # http://www.ros.org/wiki/geometry/CoordinateFrameConventions#Multi_Robot_Support
    header.frame_id = "/" + parent_name + "/" + component_name

    points = component_instance.local_data['3D_points']
    pc2 = point_cloud2.create_cloud_xyz32(header, points,
                                          component_instance.image_width *
                                          component_instance.image_height)

    for topic in self._topics:
        # publish the message on the correct topic
        if str(topic.name) == str('/' + parent_name + '/' + component_name):
            topic.publish(pc2)

    self._seq += 1
