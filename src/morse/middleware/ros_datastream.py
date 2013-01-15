import logging; logger = logging.getLogger("morse." + __name__)
import re
import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('std_msgs')
import rospy
import morse.core.datastream
from morse.core import blenderapi

from std_msgs.msg import String, Header
from morse.middleware.ros.tfMessage import tfMessage

class ROS(morse.core.datastream.Datastream):
    """ Handle communication between Blender and ROS."""

    def __init__(self):
        """ Initialize the network and generate a ROS node."""
        logger.info("ROS datastream interface initialization")
        super(self.__class__, self).__init__()
        self._topics = {}
        self._properties = {}
        self._sequence = 0
        logger.info("ROS datastream interface initialized")

    def __del__(self):
        """ Close all open ROS connections. """
        logger.info("Shutting down ROSNode...")

    def finalize(self):
        """ Kill the morse rosnode."""
        rospy.signal_shutdown("MORSE Shutdown")
        logger.info("ROS datastream: MORSE ROSNode has been killed.")

    def register_component(self, component_name, component_instance, mw_data):
        """ Generate a new topic to publish the data

        The name of the topic is composed of the robot and sensor names.
        Only useful for sensors.
        """

        logger.info("========== Registering component =================")
        parent_name = component_instance.robot_parent.blender_obj.name

        # Extract the information for this datastream interface
        # This will be tailored for each middleware according to its needs
        # This is specified in the component_config.py in Blender: [mw_data[0], mw_data[1]]
        function_name = mw_data[1]
        logger.info(" ######################## %s" % parent_name)
        logger.info(" ######################## %s" % component_name)

        # Init MORSE ROSNode
        rospy.init_node('morse', log_level=rospy.DEBUG, disable_signals=True)
        function = self._check_function_exists(function_name)

        # The function exists within this class,
        #  so it can be directly assigned to the instance
        if function != None:

            # Add data publish functions to output_functions
            if function_name == "post_message":
                # Generate one publisher and one topic for each component that is a sensor and uses post_message
                self.register_publisher(component_instance, function, String)

            # Read Strings from a rostopic
            elif function_name == "read_message":
                self.register_subscriber(component_instance, function, String, self.callback)

            else:
                #Add external module
                self._add_method(mw_data, component_instance)

        else:
            # If there is no such function in this module,
            #  try importing from another one
            try:
                # Insert the method in this class
                function = self._add_method(mw_data, component_instance)
            except IndexError as detail:
                logger.error("Method '%s' is not known, and no external module"\
                             "has been specified. Check the 'component_config.py'"\
                             " file for typos" % function_name)
                return

        logger.info("Component registered")

    def register_publisher_tf(self):
        if "tf" not in self._topics:
            self.register_publisher_name_class("/tf", tfMessage)

    def register_publisher_name_class(self, topic_name, ros_class):
        self._topics[topic_name] = rospy.Publisher(topic_name, ros_class)

    def register_publisher(self, component_instance, function, ros_class):
        component_instance.output_functions.append(function)
        topic_name = self.get_topic_name(component_instance)
        # Generate a publisher for the component
        self.register_publisher_name_class(topic_name, ros_class)
        logger.info('ROS publisher for %s initialized'%component_instance.name())

    def register_subscriber(self, component_instance, function, ros_class, callback):
        component_instance.input_functions.append(function)
        topic_name = self.get_topic_name(component_instance)
        # Generate a publisher for the component
        self._topics[topic_name] = rospy.Subscriber(topic_name, ros_class, callback, component_instance)
        logger.info('ROS subscriber for %s initialized'%component_instance.name())

    def get_topic_name(self, component_instance):
        component_name = component_instance.blender_obj.name
        # robot.001.sensor.001 = robot001.sensor001
        topic = re.sub(r'\.([0-9]+)', r'\1', component_name)
        return '/' + topic.replace('.', '/')

    def get_property(self, component_instance, name):
        component_name = component_instance.blender_obj.name
        if component_name in self._properties:
            return self._properties[component_instance.blender_obj.name][name]
        else:
            return None

    def set_property(self, component_instance, name, value):
        component_name = component_instance.blender_obj.name
        if component_name not in self._properties:
            self._properties[component_name] = {}
        self._properties[component_name][name] = value

    # Generic publish method
    def publish(self, message, component_instance):
        """ Publish the data on the rostopic
        """
        topic_name = self.get_topic_name(component_instance)
        self.publish_topic(message, topic_name)

    # Generic publish method
    def publish_topic(self, message, topic_name):
        """ Publish the data on the rostopic
        """
        self._topics[topic_name].publish(message)
        self._sequence += 1

    def get_ros_header(self, component_instance):
        header = Header()
        header.stamp = rospy.Time.now()
        header.seq = self._sequence
        # http://www.ros.org/wiki/geometry/CoordinateFrameConventions#Multi_Robot_Support
        header.frame_id = self.get_topic_name(component_instance)
        return header

    # Post string messages
    def post_message(self, component_instance):
        """ Publish the data on the rostopic
        """
        # XXX ugly, is this used somewhere ? Kept for compatibility
        data_dump = ", ".join([str(data) for data in component_instance.local_data.values()])
        self.publish(data_dump)
        # TODO self.publish(json.dumps(component_instance.local_data))

    # Callback function for reading String messages
    def callback(self, data, component_instance):
        """ Called as soon as messages are published on the specific topic
        """
        logger.info("Received String message %s on topic %s" % \
                    (data.data.decode("utf-8"), # String message decode
                     self.get_topic_name(component_instance)))

    # NOTE: This is a dummy function that is executed for every actuator.
    #       Since ROS uses the concept of callbacks, it does nothing ...
    def read_message(self, component_instance):
        """ dummy function for String-messages (could maybe be removed later)
        """

    def ros_memoryview_patched(self):
        ssr = blenderapi.getssr()
        return "ros_memoryview_patched" in ssr and ssr["ros_memoryview_patched"]
