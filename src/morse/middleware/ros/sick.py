import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('sensor_msgs')
import math
from sensor_msgs.msg import LaserScan
from morse.middleware.ros import ROSPublisher

class LaserScanPublisher(ROSPublisher):

    def initialize(self):
        ROSPublisher.initialize(self, LaserScan)
        self.frame_id = self.kwargs.get('frame_id', '/base_laser_link')

    def default(self, ci='unused'):
        """ Publish the data of the sick sensor as a ROS LaserScan message """
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
