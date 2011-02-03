import sys
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('geometry_msgs'); roslib.load_manifest('rosgraph_msgs'); roslib.load_manifest('sensor_msgs'); roslib.load_manifest('nav_msgs')
import roscpp
import rospy
import array
import morse.helpers.middleware
import GameLogic
import std_msgs
import geometry_msgs
import os
if GameLogic.pythonVersion < 3:
    import Mathutils as mathutils
else:
    import mathutils

from std_msgs.msg import String
from geometry_msgs.msg import Pose2D
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
        
class ROSClass(morse.helpers.middleware.MorseMiddlewareClass):
    """ Handle communication between Blender and ROS."""
      
    def __init__(self, obj, parent=None):
        """ Initialize the network and generate a ROS node."""
        print ("################# Initializing ROS middleware ###################")
        super(self.__class__,self).__init__(obj, parent)
        self._topics = []
        print ("################# ROS middleware initialized ####################")
        
        
    def __del__(self):
        """ Close all open ROS connections. """
        print("Shutting down ROSNode...")

    
    def finalize(self):
        """ Kill the morse rosnode."""
        rospy.signal_shutdown("ROSPy Shutdown")
        print ('ROS Mid: Morse Rosnode has been killed.')
        
    def register_component(self, component_name, component_instance, mw_data):
        """ Generate a new topic to publish the data

        The name of the topic is composed of the robot and sensor names.
        Only useful for sensors.
        """
        
        print("========== Registering component =================")
        parent_name = component_instance.robot_parent.blender_obj.name

        # Extract the information for this middleware
        # This will be tailored for each middleware according to its needs
        # This is specified in the component_config.py in Blender: [mw_data[0], mw_data[1]]
        function_name = mw_data[1]

        try:
            # Get the reference to the function
            function = getattr(self, function_name)
        except AttributeError as detail:
            print ("ERROR: %s. Check the 'component_config.py' file for typos" % detail)
            return

        print (" ######################## %s"%parent_name)
        print (" ######################## %s"%component_name )
        
        # Init MORSE-node in ROS
        rospy.init_node('morse')
        
        # Add data publish functions to output_functions
        if function_name == "post_message":
            component_instance.output_functions.append(function)
            # Generate one publisher and one topic for each component that is a sensor and uses post_message 
            self._topics.append(rospy.Publisher(parent_name + "/" + component_name, String))
        
        if function_name == "post_2DLaserScan":
            component_instance.output_functions.append(function)
            # Generate one publisher and one topic for each component that is a sensor and uses post_message 
            self._topics.append(rospy.Publisher(parent_name + "/" + component_name, LaserScan))
            
        if function_name == "post_odometry":
            component_instance.output_functions.append(function)
            # Generate one publisher and one topic for each component that is a sensor and uses post_message 
            self._topics.append(rospy.Publisher(parent_name + "/" + component_name, Odometry))    
            
        # Generate a Subscription and one topic for each actuator
        
        # Read Pose2D from a topic (for waypoint actuator)
        if function_name == "read_waypoint":
            component_instance.input_functions.append(function)
            func_wp = getattr(self, "callback_wp")
            print("component_instance is: %s"%component_instance)
            self._topics.append(rospy.Subscriber(parent_name + "/" + component_name, Pose2D, func_wp, component_instance))
        # Read Strings from a rostopic    
        if function_name == "read_message":
            component_instance.input_functions.append(function)
            func = getattr(self, "callback")
            self._topics.append(rospy.Subscriber(parent_name + "/" + component_name, String, func, component_instance))
                
        print("Component registered")

    def post_message(self, component_instance):
        """ Publish the data on the rostopic
		"""
        parent_name = component_instance.robot_parent.blender_obj.name
        # iterate through all topics and publish their data
        for topic in self._topics: 
            message = str("")
            #iterate through all objects of the component_instance and create one string
            for variable, data in component_instance.local_data.items():
                message = message + str(data) + ", "
            # publish the message on the correct topic    
            if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
                topic.publish(message)
    
    # NOTE: Posting 2D Laserscans is still experimental            
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
                
    def post_odometry(self, component_instance):
        """ Publish the data on the rostopic
		"""
        parent_name = component_instance.robot_parent.blender_obj.name
        odometry = Odometry()
        
        odometry.pose.pose.position.x = component_instance.position_3d.x
        odometry.pose.pose.position.y = component_instance.position_3d.y
        odometry.pose.pose.position.z = component_instance.position_3d.z
        odometry.pose.pose.orientation.x = component_instance.position_3d.roll
        odometry.pose.pose.orientation.x = component_instance.position_3d.pitch
        odometry.pose.pose.orientation.x = component_instance.position_3d.yaw
        
        # Covariance matrix is identity matrix when using ground truth (NOTE: Could also be 0-matrix?)
        odometry.pose.covariance[0] = 1
        odometry.pose.covariance[7] = 1
        odometry.pose.covariance[14] = 1
        odometry.pose.covariance[21] = 1
        odometry.pose.covariance[28] = 1
        odometry.pose.covariance[35] = 1
        
        for topic in self._topics: 
            message = odometry
            # publish the message on the correct topic    
            if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
                topic.publish(odometry)

    def callback(self, data, component_instance):
        """ this function is called as soon as messages are published on the specific topic """
        print("Received String-message %s on topic %s"%(data.data.decode("utf-8"), str("/" + component_instance.robot_parent.blender_obj.name + "/" + component_instance.blender_obj.name)))
        
    def callback_wp(self, data, component_instance):
        """ this function is called as soon as Pose2Ds are published on the specific topic """
        print("Received Pose2D: < %s, %s, %s > on topic %s"%(data.x, data.y, data.theta, str("/" + component_instance.robot_parent.blender_obj.name + "/" + component_instance.blender_obj.name)))
        component_instance.local_data["x"] = data.x
        component_instance.local_data["y"] = data.y
        component_instance.local_data["z"] = data.theta
			
    # NOTE: These are the dummy functions that are executed for every actuator. Since ROS uses the concept of callbacks, 
    # these functions actually do nothing at the moment and maybe can be removed later...    
    def read_message(self, component_instance):
        """ dummy function for String-messages (could maybe be removed later)"""
    
    def read_waypoint(self, component_instance):
        """ dummy function for Waypoints (could mybe be removed later) """
