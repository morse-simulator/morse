import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('sensor_msgs'); roslib.load_manifest('rosgraph_msgs')  
import rospy
import math
import std_msgs
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud

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
    if mw_data[1] == "post_2DLaserScan":  
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, LaserScan))
    else:
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, PointCloud)) 

def post_2DLaserScan(self, component_instance):
    """ Publish the data on the rostopic """
    laserHeader = std_msgs.msg.Header()
    laserHeader.stamp = rospy.Time.now()
    laserHeader.frame_id = '/base_laser_link'
    num_readings = component_instance.blender_obj['scan_window'] / component_instance.blender_obj['resolution']
    max_angle = component_instance.blender_obj['scan_window'] * ( math.pi / 360 )
    min_angle = max_angle * (-1)
    angle_incr = ((component_instance.blender_obj['scan_window'] / num_readings) * (math.pi / 180)) 
    laser_maxrange =  component_instance.blender_obj['laser_range']

    # Note: Scan time and laser frequency are chosen as standard values        
    laser_frequency = 40
    laserscan = LaserScan(header = laserHeader, angle_min = min_angle, angle_max = max_angle, angle_increment = angle_incr, time_increment = ((1 / laser_frequency) / (num_readings)), scan_time = 1.0, range_min = 0.3, range_max = laser_maxrange, ranges = component_instance.local_data['range_list'])
        
    parent_name = component_instance.robot_parent.blender_obj.name
    for topic in self._topics: 
        message = laserscan
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(laserscan)
          
#WARNING: posting 2D-Pointclouds does NOT work at the moment due to Python3 encoding errors
def post_2DPointCloud(self, component_instance):
    """ Publish the data on the rostopic 	"""
    pcHeader = std_msgs.msg.Header()
    pcHeader.stamp = rospy.Time.now()
    pcHeader.frame_id = '/base_laser_link'
    
    pointcloud = PointCloud(header = pcHeader, points = component_instance.local_data['point_list'])
    
    parent_name = component_instance.robot_parent.blender_obj.name
    for topic in self._topics: 
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(pointcloud)
