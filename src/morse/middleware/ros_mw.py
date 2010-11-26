import sys
import rospy
import array
import morse.helpers.middleware
import GameLogic

class MorseROSClass(morse.helpers.middleware.MorseMiddlewareClass):
    """ Handle communication between Blender and ROS."""
    
    def __init__(self, obj, parent=None):
        """ Initialize the network and generate a ROS node."""
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)
        # TODO: Somehow connect to roscore here and generate a new rosnode
        
    def __del__(self):
        """ Close all open ROS connections. """
        # TODO: Cleanup connections if neccessary
        
    def register_component(self, component_name, component_instance, mw_data):
        """ Generate a new topic to publish the data

        The name of the topic is composed of the robot and sensor names.
        Only useful for sensors.
        """
        parent_name = component_instance.robot_parent.blender_obj.name

        # Extract the information for this middleware
        # This will be tailored for each middleware according to its needs
        # This is specified in the component_config.py in Blender: [mw_data[]0, mw_data[1]]
        function_name = mw_data[1]

        try:
            # Get the reference to the function
            function = getattr(self, function_name)
        except AttributeError as detail:
            print ("ERROR: %s. Check the 'component_config.py' file for typos" % detail)
            return

        # Data publish functions
        # I didnt exactly get why the function has to be appended here... ???
        if function_name == "publish_data":
            component_instance.output_functions.append(function)

        # TODO: Prepare a ros topic to publish the data

def publish_data(self, component_instance):
        """ Publish the data on the rostopic"""
        
        # TODO: somehow receive modified_data from the sensor/actuator and publish the msg on the topic
        
def receive_data(self, component_instance):
        """ subscribe to rostopic and receive data"""
        
        # TODO: somehow subscribe to ros topic and read messages from there
