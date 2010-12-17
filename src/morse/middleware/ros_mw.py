import sys
sys.path += ["/usr/wiss/kargm/ros/ros/core/roslib/src"]
import roslib; roslib.load_manifest('morse')
import rospy
import array
import morse.helpers.middleware
import GameLogic


from std_msgs.msg import String


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
        # TODO: Cleanup connections if neccessary
        
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
        
        # Add data publish functions to output_functions
        if function_name == "post_message":
            component_instance.output_functions.append(function)
        
        # Generate one publisher and one topic for each component (currently only with String-message-type)
        self._topics.append(rospy.Publisher(parent_name + "/" + component_name, String))
        
        # Init MORSE-node in ROS
        rospy.init_node('morse')
        print("Component registered")

    def post_message(self, component_instance):
        """ Publish the data on the rostopic"""
        
        parent_name = component_instance.robot_parent.blender_obj.name
        # iterate through all topics and publish their data
        
        for topic in self._topics: 
            message = str("")
            #iterate through all objects of the component_instance and create one string
            for obj in component_instance.modified_data:
                message = message + str(obj) + ", "
            # publish the message on the right topic    
            if str(topic.name) == str("/" + parent_name + "/" + component_instance.blender_obj.name):
                topic.publish(message)
        
    def receive_data(self, component_instance):
        """ subscribe to rostopic and receive data"""
        
        # TODO: somehow subscribe to ros topic and read messages from there
        # basically: write code from listener tutiorial here...
