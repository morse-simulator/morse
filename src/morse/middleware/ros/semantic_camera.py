import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('std_msgs');  roslib.load_manifest('geometry_msgs')
import rospy
from std_msgs.msg import String
from morse.middleware.ros.tfMessage import tfMessage
from geometry_msgs.msg import TransformStamped

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    self.register_publisher(component_instance, function, String)
    self.register_publisher_tf()

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
    self.publish_topic(tfm, "/tf")

def post_string(self, component_instance):
    """ Publish the data of the semantic camera as a string-message with newlines (for better visualization in console).

    """
    string = String()
    for obj in component_instance.local_data['visible_objects']:
        # if object has no description, set to '-'
        if obj['description'] == '':
            description = '-'

        # send tf-frame for every object
        sendTransform(self, obj['position'], obj['orientation'], rospy.Time.now(), str(obj['name']), "/map")

        # Build string from name, description, location and orientation in the global world frame
        string.data += "[" + str(obj['name']) + ", " + description + ", " + \
                       str(obj['position']) + ", " + str(obj['orientation']) + " ]\n"

    self.publish(string, component_instance)

def post_lisp_code(self, component_instance):
    """ Publish the data of the semantic camera as a string-message that contains a lisp-list. This function was designed for the use with CRAM and the Adapto group

    """
    string = String()
    string.data = "("
    for obj in component_instance.local_data['visible_objects']:
        # if object has no description, set to '-'
        if obj['description'] == '':
            description = '-'

        # send tf-frame for every object
        sendTransform(self, obj['position'], obj['orientation'], rospy.Time.now(), str(obj['name']), "/map")

        # Build string from name, description, location and orientation in the global world frame
        string.data += "(" + str(obj['name']) + " " + description + " " + \
                       str(obj['position'].x) + " " + \
                       str(obj['position'].y) + " " + \
                       str(obj['position'].z) + " " + \
                       str(obj['orientation'].x) + " " + \
                       str(obj['orientation'].y) + " " + \
                       str(obj['orientation'].z) + " " + \
                       str(obj['orientation'].w) + ")"

    string.data += ")"
    self.publish(string, component_instance)
