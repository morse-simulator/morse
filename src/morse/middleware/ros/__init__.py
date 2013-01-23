import logging; logger = logging.getLogger("morse." + __name__)
import re
import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('std_msgs')
import rospy

from std_msgs.msg import String, Header
from morse.middleware.ros.tfMessage import tfMessage

from morse.middleware import AbstractDatastream

class ROS(AbstractDatastream):
    topic_tf = None

    def initalize(self):
        # Initialize MORSE-ROS-node. If already initialized, does nothing
        rospy.init_node('morse', log_level=rospy.DEBUG, disable_signals=True)

        logger.info("ROS datastream initalize %s"%self)
        self.sequence = 0 # for ROS msg Header
        self.topic = None

        if 'topic' in self.kwargs:
            self.topic_name = self.kwargs['topic']
        else:
            self.topic_name = self.get_topic_name()

        self.frame_id = self.kwargs.get('frame_id', self.topic_name)

    def get_topic_name(self):
        # robot.001.sensor.001 = robot001.sensor001
        topic_name = re.sub(r'\.([0-9]+)', r'\1', self.component_name)
        # '/robot001/sensor001'
        return '/' + topic_name.replace('.', '/')

    def finalize(self):
        """ Shutdown the MORSE-ROS-node."""
        # Unregister the topic
        self.topic.unregister()
        ROS.topic_tf.unregister()
        rospy.signal_shutdown("MORSE Shutdown")
        logger.info("ROS datastream finalize %s"%self)


class ROSPublisher(ROS):

    def initalize(self, ros_class):
        ROS.initalize(self)
        topic_name = self.topic_name
        if 'topic_suffix' in self.kwargs: # used for /robot/camera/image
            topic_name += self.kwargs['topic_suffix']
        # Generate a publisher for the component
        self.topic = rospy.Publisher(topic_name, ros_class)
        logger.info('ROS publisher initialized for %s'%self)

    def register_tf(self):
        if not ROS.topic_tf:
            ROS.topic_tf = rospy.Publisher("/tf", tfMessage)

    def get_ros_header(self):
        header = Header()
        header.stamp = rospy.Time.now()
        header.seq = self.sequence
        # http://www.ros.org/wiki/geometry/CoordinateFrameConventions#Multi_Robot_Support
        header.frame_id = self.frame_id
        return header

    # Generic publish method
    def publish(self, message):
        """ Publish the data on the rostopic
        """
        self.topic.publish(message)
        self.sequence += 1

    # TF publish method
    def publish_tf(self, message):
        """ Publish the TF data on the rostopic
        """
        ROS.topic_tf.publish(message)


class ROSReader(ROS):

    def initalize(self, ros_class):
        ROS.initalize(self)
        self.message = None
        # Generate a subscriber for the component
        self.topic = rospy.Subscriber(self.topic_name, ros_class, self.callback)
        logger.info('ROS subscriber initialized for %s'%self)

    def callback(self, message):
        if self.message is None:
            self.message = message

    def default(self, ci=None):
        # If a new message has been received
        if self.message:
            # Update local_data
            self.update(self.message)
            # Free message for new reception
            self.message = None
            # Tell MORSE that we can apply modifiers
            return True

        return False

    def update(self, message):
        """ Update `local_data` with :param message:

        Called when component.default_action is triggered
        and a new message was received
        """
        pass

#
# Example (String)
#

class StringPublisher(ROSPublisher):

    def initalize(self):
        ROSPublisher.initalize(self, String)

    # Post string messages
    def default(self, ci=None):
        """ Publish the `local_data` on the topic
        """
        self.publish(repr(self.component_instance.local_data))


class StringReader(ROSReader):

    def initalize(self):
        ROSReader.initalize(self, String)

    def update(self, message):
        logger.info("Received String message %s on topic %s" % \
                    (message.data.decode("utf-8"), # String message decode
                     self.topic_name))
