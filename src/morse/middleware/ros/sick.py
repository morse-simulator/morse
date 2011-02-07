import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('sensor_msgs'); roslib.load_manifest('rosgraph_msgs')  
import rospy
import std_msgs
from sensor_msgs.msg import LaserScan

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
    self._topics.append(rospy.Publisher(parent_name + "/" + component_name, LaserScan))

#Note: posting 2D Laserscans is still experimental!
def post_2DLaserScan(self, component_instance):
        """ Publish the data on the rostopic
		"""
        parent_name = component_instance.robot_parent.blender_obj.name
        # iterate through all topics and publish their data
        laserHeader = std_msgs.msg.Header()
        laserHeader.frame_id = 'base_link'
        laserscan = LaserScan(header = laserHeader, angle_min = 0, angle_max = 1.14, angle_increment = 0.025, time_increment = 0.1, scan_time = 2.0, range_min = 0.1, range_max = 80.00, ranges = component_instance.local_data['ranges_list'])			
        for topic in self._topics: 
            message = laserscan
            # publish the message on the correct topic    
            if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
                topic.publish(laserscan)
