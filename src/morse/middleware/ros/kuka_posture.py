import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('sensor_msgs')
import rospy
import std_msgs
from sensor_msgs.msg import JointState
import GameLogic
import math
import mathutils

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
    self._topics.append(rospy.Publisher(parent_name + "/" + component_name, JointState))
        
    print('######## ROS JOINTSTATE PUBLISHER INITIALIZED ########')

def post_jointState(self, component_instance):
    """ Publish the data of the Odometry-sensor as a ROS-Pose message
    """
    parent_name = component_instance.robot_parent.blender_obj.name
    js = JointState()

    js.name = 'seg0', 'seg1', 'seg2', 'seg3', 'seg4', 'seg5', 'seg6'

    js.position = [component_instance.local_data['seg0'], component_instance.local_data['seg1'], component_instance.local_data['seg2'], component_instance.local_data['seg3'], component_instance.local_data['seg4'], component_instance.local_data['seg5'], component_instance.local_data['seg6']] 
               
    for topic in self._topics: 
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name): 
            topic.publish(js)
