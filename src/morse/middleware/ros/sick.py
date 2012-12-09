import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('sensor_msgs')
import math
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_publisher(component_instance, function, get_ros_class(mw_data[1]))

def get_ros_class(method_name):
    dict_method_class = {
        'post_2DLaserScan': LaserScan,
        'post_2DPointCloud': PointCloud,
    }
    return dict_method_class[method_name]

def post_2DLaserScan(self, component_instance):
    """ Publish the data on the rostopic """
    laserscan = LaserScan()
    laserscan.header = self.get_ros_header(component_instance)
    laserscan.header.frame_id = '/base_laser_link'

    # Note: Scan time and laser frequency are chosen as standard values
    laser_frequency = 40
    laser_maxrange =  component_instance.blender_obj['laser_range']
    num_readings = component_instance.blender_obj['scan_window'] / component_instance.blender_obj['resolution']

    laserscan.angle_max = component_instance.blender_obj['scan_window'] * ( math.pi / 360 )
    laserscan.angle_min = max_angle * (-1)
    laserscan.angle_increment = ((component_instance.blender_obj['scan_window'] / num_readings) * (math.pi / 180))
    laserscan.time_increment = ((1 / laser_frequency) / (num_readings))
    laserscan.scan_time = 1.0
    laserscan.range_min = 0.3
    laserscan.range_max = laser_maxrange
    laserscan.ranges = component_instance.local_data['range_list']

    self.publish(laserscan, component_instance)

#WARNING: posting 2D-Pointclouds does NOT work at the moment due to Python3 encoding errors
def post_2DPointCloud(self, component_instance):
    """ Publish the data on the rostopic """
    pointcloud = PointCloud()
    pointcloud.header = self.get_ros_header(component_instance)
    pointcloud.header.frame_id = '/base_laser_link'

    pointcloud.points = component_instance.local_data['point_list']

    self.publish(pointcloud, component_instance)
