import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('std_msgs')
import rospy
from std_msgs.msg import String

class SemanticCameraPublisher(ROSPublisherTF):

    def initalize(self):
        ROSPublisherTF.initalize(self, String)

    def default(self, ci='unused'):
        """ Publish the data of the semantic camera as a ROS String message with newlines (for better visualization in console). """
        string = String()
        for obj in self.data['visible_objects']:
            # if object has no description, set to '-'
            if obj['description'] == '':
                description = '-'

            # send tf-frame for every object
            self.sendTransform(self, obj['position'], obj['orientation'], \
                               rospy.Time.now(), str(obj['name']), "/map")

            # Build string from name, description, location and orientation in the global world frame
            string.data += "[" + str(obj['name']) + ", " + description + ", " + \
                           str(obj['position']) + ", " + str(obj['orientation']) + " ]\n"

        self.publish(string)


class SemanticCameraPublisherLisp(ROSPublisherTF):

    def initalize(self):
        ROSPublisherTF.initalize(self, String)

    def default(self, ci='unused'):
        """ Publish the data of the semantic camera as a ROS String message that contains a lisp-list.

        This function was designed for the use with CRAM and the Adapto group
        """
        string = String()
        string.data = "("
        for obj in self.data['visible_objects']:
            # if object has no description, set to '-'
            if obj['description'] == '':
                description = '-'

            # send tf-frame for every object
            self.sendTransform(self, obj['position'], obj['orientation'], \
                               rospy.Time.now(), str(obj['name']), "/map")

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
