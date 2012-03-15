import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy'); roslib.load_manifest('std_msgs');  roslib.load_manifest('geometry_msgs')
import rospy
import std_msgs
import bge
import math
import mathutils

from std_msgs.msg import String
from morse.middleware.ros.tfMessage import tfMessage
from geometry_msgs.msg import TransformStamped

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
    self._topics.append(rospy.Publisher(parent_name + "/" + component_name, String))
    
    logger.info('######## ROS SEMANTIC CAMERA PUBLISHER INITIALIZED ########')

def sendTransform(self, translation, rotation, time, child, parent):
    """
    :param translation: the translation of the transformtion as a tuple (x, y, z)
    :param rotation: the rotation of the transformation as a tuple (x, y, z, w)
    :param time: the time of the transformation, as a rospy.Time()
    :param child: child frame in tf, string
    :param parent: parent frame in tf, string
    
    Broadcast the transformation from tf frame child to parent on ROS topic ``"/tf"``.
    """
    
    t = TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = time
    t.child_frame_id = child
    t.transform.translation.x = translation[0]
    t.transform.translation.y = translation[1]
    t.transform.translation.z = translation[2]
    
    t.transform.rotation.x = rotation[0]
    t.transform.rotation.y = rotation[1]
    t.transform.rotation.z = rotation[2]
    t.transform.rotation.w = rotation[3]
    
    tfm = tfMessage([t])
    self.pub_tf.publish(tfm)

def post_string(self, component_instance):
    """ Publish the data of the semantic camera as a string-message with newlines (for better visualization in console).

    """
    parent_name = component_instance.robot_parent.blender_obj.name
    string = String()
               
    for topic in self._topics: 
        message = str("")
        #iterate through all objects of the component_instance and create one string
        for obj in component_instance.local_data['visible_objects']:
            #if object has no description, set to '-'
            if obj['description'] == '':
                description = '-'

            # send tf-frame for every object
            sendTransform(self, obj['position'], obj['orientation'],rospy.Time.now(), str(obj['name']), "/map")

            # Build string from name, description, location and orientation in the global world frame
            message = message + "[" + str(obj['name']) + ", " + description + ", " + str(obj['position']) + ", " + str(obj['orientation']) + " ]\n"    
            string.data = message
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(string)

def post_lisp_code(self, component_instance):
    """ Publish the data of the semantic camera as a string-message that contains a lisp-list. This function was designed for the use with CRAM and the Adapto group

    """
    parent_name = component_instance.robot_parent.blender_obj.name
    string = String()
               
    for topic in self._topics: 
        message = str("(")

        #iterate through all objects of the component_instance and create one string in lisp-list format
        for obj in component_instance.local_data['visible_objects']:
            #if object has no description, set to '-'
            if obj['description'] == '':
                description = '-'
            # Build string from name, description, location and orientation in the global world frame
            message = message + "(" + str(obj['name']) + " " + description + " " + str(obj['position'].x) + " " + str(obj['position'].y) + " " + str(obj['position'].z) + " " + str(obj['orientation'].x) + " " + str(obj['orientation'].y) + " " + str(obj['orientation'].z) + " " + str(obj['orientation'].w) + ")"
        
        string.data = message + ")"
        # publish the message on the correct topic    
        if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
            topic.publish(string)
