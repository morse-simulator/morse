import logging; logger = logging.getLogger("morse.ros")
import re
import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('std_msgs'); roslib.load_manifest('geometry_msgs')
import rospy

from std_msgs.msg import String, Header
from geometry_msgs.msg import TransformStamped

from morse.middleware.ros.tfMessage import tfMessage
from morse.middleware import AbstractDatastream

try:
    import mathutils
except ImportError:
    # running outside Blender
    mathutils = None

class classproperty(object):
    def __init__(self, fget):
        self.fget = fget
    def __get__(self, owner_self, owner_cls):
        return self.fget(owner_cls)

class AbstractROS(AbstractDatastream):
    """ Base class for all ROS Publishers and Subscribers """
    ros_class = Header # default

    @classproperty
    def _type_name(cls):
        # returns the standard ROS message name from its Python class
        return "%s/%s"%(cls.ros_class.__module__.split('.')[0], cls.ros_class.__name__)

    @classproperty
    def _type_url(cls):
        # used to generate documentation
        return "http://ros.org/doc/api/%s/html/msg/%s.html"%tuple(cls._type_name.split('/'))

    def initialize(self):
        # Initialize MORSE-ROS-node. If already initialized, does nothing
        rospy.init_node('morse', log_level=rospy.DEBUG, disable_signals=True)

        logger.info("ROS datastream initialize %s"%self)
        self.topic = None

        if 'topic' in self.kwargs:
            self.topic_name = self.kwargs['topic']
        else:
            self.topic_name = self.get_topic_name()

    def get_topic_name(self):
        # robot.001.sensor.001 = robot001.sensor001
        topic_name = re.sub(r'\.([0-9]+)', r'\1', self.component_name)
        # '/robot001/sensor001'
        return '/' + topic_name.replace('.', '/')

    def finalize(self):
        """ Shutdown the MORSE-ROS-node."""
        # Unregister the topic if one exists
        if self.topic:
            self.topic.unregister()
        rospy.signal_shutdown("MORSE Shutdown")
        logger.info("ROS datastream finalize %s"%self)


class ROSPublisher(AbstractROS):
    """ Base class for all ROS Publishers """
    default_frame_id = 'USE_TOPIC_NAME'

    def initialize(self):
        AbstractROS.initialize(self)
        topic_name = self.topic_name
        if 'topic_suffix' in self.kwargs: # used for /robot/camera/image
            topic_name += self.kwargs['topic_suffix']
        # do not create a topic if no ros_class set (for TF publish only)
        if self.ros_class != Header:
            # Generate a publisher for the component
            self.topic = rospy.Publisher(topic_name, self.ros_class)
        if self.default_frame_id is 'USE_TOPIC_NAME': # morse convention
            self.frame_id = self.kwargs.get('frame_id', self.topic_name)
        else: # default_frame_id was overloaded in subclass
            self.frame_id = self.kwargs.get('frame_id', self.default_frame_id)
        self.sequence = 0 # for ROS msg Header
        logger.info('ROS publisher initialized for %s'%self)

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


class ROSPublisherTF(ROSPublisher):
    """ Base class for all ROS Publishers with TF support """
    topic_tf = None

    def initialize(self):
        ROSPublisher.initialize(self)
        if not ROSPublisherTF.topic_tf:
            ROSPublisherTF.topic_tf = rospy.Publisher("/tf", tfMessage)

    def finalize(self):
        ROSPublisher.finalize(self)
        ROSPublisherTF.topic_tf.unregister()

    def get_local_transform(self):
        """ Get the transformation relative to the robot origin

        Return the local position, orientation and scale of this components
        """
        obj = self.component_instance.bge_object
        # XXX not same as return obj.localTransform.decompose()
        return (obj.localPosition, obj.localOrientation.to_quaternion(), obj.localScale)

    def send_transform_robot(self, time=None, child=None, parent=None):
        """ Send the transformation relative to the robot

        :param time: default now
        :param child: default topic_name or 'frame_id' in kwargs
        :param parent: default 'base_link' or 'parent_frame_id' in kwargs
        """
        translation, rotation, _ = self.get_local_transform()
        if not time:
            time = rospy.Time.now()
        if not child:
            # our frame_id (component frame)
            child = self.frame_id
        if not parent:
            # get parent frame_id (aka. the robot)
            parent = self.kwargs.get('parent_frame_id', 'base_link')
        #rospy.loginfo("t:%s,r:%s"%(str(translation), str(rotation)))
        # send the transformation
        self.sendTransform(translation, rotation, time, child, parent)

    # TF publish method
    def publish_tf(self, message):
        """ Publish the TF data on the rostopic
        """
        ROSPublisherTF.topic_tf.publish(message)

    def sendTransform(self, translation, rotation, time, child, parent):
        """
        :param translation: the translation of the transformtion as geometry_msgs/Vector3
        :param rotation: the rotation of the transformation as a geometry_msgs/Quaternion
        :param time: the time of the transformation, as a rospy.Time()
        :param child: child frame in tf, string
        :param parent: parent frame in tf, string

        Broadcast the transformation from tf frame child to parent on ROS topic ``"/tf"``.
        """

        t = TransformStamped()
        t.header.frame_id = parent
        t.header.stamp = time
        t.child_frame_id = child
        t.transform.translation = translation
        t.transform.rotation = rotation

        tfm = tfMessage([t])

        self.publish_tf(tfm)


class ROSReader(AbstractROS):
    """ Base class for all ROS Subscribers """

    def initialize(self):
        AbstractROS.initialize(self)
        self.message = None
        # Generate a subscriber for the component
        self.topic = rospy.Subscriber(self.topic_name, self.ros_class, self.callback)
        logger.info('ROS subscriber initialized for %s'%self)

    def callback(self, message):
        if self.message is None:
            self.message = message

    def default(self, ci='unused'):
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
    """ Publish a string containing a printable representation of the local data. """
    ros_class = String

    def default(self, ci='unused'):
        self.publish(repr(self.data))


class StringReader(ROSReader):
    """ Subscribe to a String topic and log its data decoded as UTF-8. """
    ros_class = String

    def update(self, message):
        logger.info("Received String message %s on topic %s" % \
                    (message.data.decode("utf-8"), # String message decode
                     self.topic_name))
