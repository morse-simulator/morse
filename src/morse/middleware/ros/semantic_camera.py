import logging; logger = logging.getLogger("morse." + __name__)
import json
import rospy
from std_msgs.msg import String
from morse.middleware.ros import ROSPublisherTF
from morse.middleware.socket_datastream import MorseEncoder

class SemanticCameraPublisher(ROSPublisherTF):
    """ Publish the data of the semantic camera as JSON in a ROS String message.
    And send TF transform between '/map' and ``object.name``.
    """
    ros_class = String

    def initialize(self):
        if not self.component_instance.relative:
            self.default_frame_id = '/map'
        ROSPublisherTF.initialize(self)

    def default(self, ci='unused'):
        for obj in self.data['visible_objects']:
            # send tf-frame for every object
            self.sendTransform(obj['position'], obj['orientation'],
                               self.get_time(), str(obj['name']), self.frame_id)
        string = String()
        string.data = json.dumps(self.data['visible_objects'], cls=MorseEncoder)
        self.publish(string)


class SemanticCameraPublisherLisp(ROSPublisherTF):
    """ Publish the data of the semantic camera as a ROS String message,
    that contains a lisp-list (each field are separated by a space).

    This function was designed for the use with CRAM and the Adapto group.
    """
    ros_class = String

    def initialize(self):
        if not self.component_instance.relative:
            self.default_frame_id = '/map'
        ROSPublisherTF.initialize(self)

    def default(self, ci='unused'):
        string = String()
        string.data = "("
        for obj in self.data['visible_objects']:
            description = obj['description'] or '-'

            # send tf-frame for every object
            self.sendTransform(obj['position'], obj['orientation'],
                               self.get_time(), str(obj['name']), self.frame_id)

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
        self.publish(string)
