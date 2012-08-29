import logging; logger = logging.getLogger("morse." + __name__)
import roslib; roslib.load_manifest('roscpp'); roslib.load_manifest('rospy');
import roscpp
import rospy
import morse.core.middleware
import std_msgs

from std_msgs.msg import String

class ROSClass(morse.core.middleware.MorseMiddlewareClass):
    """ Handle communication between Blender and ROS."""

    def __init__(self):
        """ Initialize the network and generate a ROS node."""
        logger.info("Middleware initialization")
        super(self.__class__, self).__init__()
        self._topics = []
        self._properties = {}
        logger.info("Middleware initialized")


    def __del__(self):
        """ Close all open ROS connections. """
        logger.info("Shutting down ROSNode...")


    def finalize(self):
        """ Kill the morse rosnode."""
        rospy.signal_shutdown("ROSPy Shutdown")
        logger.info('ROS Mid: Morse Rosnode has been killed.')

    def register_component(self, component_name, component_instance, mw_data):
        """ Generate a new topic to publish the data

        The name of the topic is composed of the robot and sensor names.
        Only useful for sensors.
        """

        logger.info("========== Registering component =================")
        parent_name = component_instance.robot_parent.blender_obj.name

        # Extract the information for this middleware
        # This will be tailored for each middleware according to its needs
        # This is specified in the component_config.py in Blender: [mw_data[0], mw_data[1]]
        function_name = mw_data[1]
        logger.info(" ######################## %s" % parent_name)
        logger.info(" ######################## %s" % component_name)

        # Init MORSE-node in ROS
        rospy.init_node('morse', log_level=rospy.DEBUG, disable_signals=True)
        function = self._check_function_exists(function_name)

        # The function exists within this class,
        #  so it can be directly assigned to the instance
        if function != None:

            # Add data publish functions to output_functions
            if function_name == "post_message":
                component_instance.output_functions.append(function)
                # Generate one publisher and one topic for each component that is a sensor and uses post_message 
                self._topics.append(rospy.Publisher(parent_name + "/" + component_name, String))

            # Read Strings from a rostopic    
            elif function_name == "read_message":
                component_instance.input_functions.append(function)
                func = getattr(self, "callback")
                self._topics.append(rospy.Subscriber(parent_name + "/" + component_name, String, func, component_instance))

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
                logger.error("Method '%s' is not known, and no external module has been specified. Check the 'component_config.py' file for typos" % function_name)
                return

        logger.info("Component registered")

    # Post string messages
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

    # Callback function for reading String messages
    def callback(self, data, component_instance):
        """ this function is called as soon as messages are published on the specific topic """
        logger.info("Received String-message %s on topic %s" % (data.data.decode("utf-8"), str("/" + component_instance.robot_parent.blender_obj.name + "/" + component_instance.blender_obj.name)))

    # NOTE: This is a dummy function that is executed for every actuator. Since ROS uses the concept of callbacks, it does nothing ...    
    def read_message(self, component_instance):
        """ dummy function for String-messages (could maybe be removed later)"""
