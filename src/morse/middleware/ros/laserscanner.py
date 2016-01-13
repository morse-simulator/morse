import logging; logger = logging.getLogger("morse." + __name__)
import math
import struct
import itertools
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from morse.middleware.ros import ROSPublisher, ROSPublisherTF

class LaserScanPublisher(ROSPublisher):
    """ Publish the ``range_list`` of the laser scanner. """
    ros_class = LaserScan
    default_frame_id = '/base_laser_link'

    def default(self, ci='unused'):
        laserscan = LaserScan()
        laserscan.header = self.get_ros_header()

        # Note: Scan time and laser frequency are chosen as standard values
        laser_frequency = 40 # TODO ? component_instance.frequency()
        scan_window = self.component_instance.bge_object['scan_window']
        num_readings = scan_window / self.component_instance.bge_object['resolution']

        laserscan.angle_max = scan_window * math.pi / 360
        laserscan.angle_min = laserscan.angle_max * -1
        laserscan.angle_increment = scan_window / num_readings * math.pi / 180
        laserscan.time_increment = 1 / laser_frequency / num_readings
        laserscan.scan_time = 1.0
        laserscan.range_min = 0.3
        laserscan.range_max = self.component_instance.bge_object['laser_range']
        # ROS expect the ranges to be sorted clockwise.
        # see morse.builder.sensor.LaserSensorWithArc.create_laser_arc
        # where we create the ray from -window / 2.0 to +window / 2.0
        laserscan.ranges = self.data['range_list']

        self.publish(laserscan)

class PointCloud2Publisher(ROSPublisherTF):
    """ Publish the ``point_list`` of the laser scanner. """
    ros_class = PointCloud2

    def default(self, ci='unused'):
        points = self.data['point_list']
        size = len(points)

        pc2 = PointCloud2()
        pc2.header = self.get_ros_header()
        pc2.height = 1
        pc2.width = size
        pc2.is_dense = False
        pc2.is_bigendian = False
        pc2.fields = [PointField('x', 0, PointField.FLOAT32, 1),
                      PointField('y', 4, PointField.FLOAT32, 1),
                      PointField('z', 8, PointField.FLOAT32, 1)]
        pc2.point_step = 12
        pc2.row_step = size * 12

        pc2.data = pack_xyz_float32(points)

        self.publish(pc2)
        self.send_transform_robot()

def pack_xyz_float32(points):
    flatten = itertools.chain.from_iterable(points)
    return struct.pack('%if'%len(points)*3, *flatten)
