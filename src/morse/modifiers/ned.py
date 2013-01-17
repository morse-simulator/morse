import logging; logger = logging.getLogger("morse." + __name__)
import math

from morse.core.modifier import Modifier

class MorseNEDClass(Modifier):
    """ Convert between ENU and NED coordinates. """
    
    def register_component(self, component_name, component_instance, mod_data):
        """ Add the corresponding function to a component. """
        # Extract the information for this modifier
        # This will be tailored for each middleware according to its needs
        function_name = mod_data[1]

        try:
            # Get the reference to the function
            function = getattr(self, function_name)
        except AttributeError as detail:
            logger.error("%s. Check the 'component_config.py' file for typos" % detail)
            return

        # Choose what to do, depending on the function being used
        # Data read functions
        if function_name == "ned_to_blender":
            component_instance.input_modifiers.append(function)
        # Data write functions
        elif function_name == "blender_to_ned":
            component_instance.output_modifiers.append(function)
        
        # Data read functions
        if function_name == "ned_angle_to_blender":
            component_instance.input_modifiers.append(function)
        # Data write functions
        elif function_name == "blender_to_ned_angle":
            component_instance.output_modifiers.append(function)

    def blender_to_ned(self, component_instance):
        """ Convert the coordinates from Blender to UTM reference. """
        tmp = component_instance.local_data['x']
        component_instance.local_data['x'] = component_instance.local_data['y']
        component_instance.local_data['y'] = tmp
        component_instance.local_data['z'] = -component_instance.local_data['z']


    def ned_to_blender(self, component_instance):
        """ Convert the coordinates from UTM to Blender reference. """
        tmp = component_instance.local_data['x']
        component_instance.local_data['x'] = component_instance.local_data['y']
        component_instance.local_data['y'] = tmp
        component_instance.local_data['z'] = -component_instance.local_data['z']



    def blender_to_ned_angle(self, component_instance):
        """ Convert the coordinates from Blender to UTM reference. """
        try:
            roll = math.pi/2 - component_instance.local_data['yaw']
            component_instance.local_data['yaw'] = component_instance.local_data['roll']
            component_instance.local_data['pitch'] = -component_instance.local_data['pitch']
            component_instance.local_data['roll'] = roll
        except KeyError as detail:
            logger.warning("Unable to use 'blender_to_ned_angle component %s. It does not contain the item %s in its 'local_data' dictionary" % (component_instance.bge_object.name, detail))

    def ned_angle_to_blender(self, component_instance):
        """ Convert the coordinates from UTM to Blender reference. """
        try:
            yaw = math.pi/2 - component_instance.local_data['roll']
            component_instance.local_data['pitch'] = -component_instance.local_data['pitch']
            component_instance.local_data['roll'] = component_instance.local_data['yaw']
            component_instance.local_data['yaw'] = yaw
        except KeyError as detail:
            logger.warning("Unable to use 'ned_angle_to_blender' on component %s. It does not contain the item %s in its 'local_data' dictionary" % (component_instance.bge_object.name, detail))
