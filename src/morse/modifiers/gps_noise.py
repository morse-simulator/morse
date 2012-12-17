import logging; logger = logging.getLogger("morse." + __name__)
import morse.modifiers.gaussian

from morse.core.modifier import Modifier

class MorseGPSNoiseClass(Modifier):

    dev = 0

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

        if function_name == "noisify":
            component_instance.output_modifiers.append(function)
        else:
            logger.warning("Unknown function name for GPS Noise modifier. Check component_config.py file.")

        # Extract the Modifier parameters
        try:
            self.dev = mod_data[2]
        except:
            pass


    def noisify(self, component_instance):
        for variable, data in component_instance.local_data.items():
            component_instance.local_data[variable] = morse.modifiers.gaussian.gaussian(self.dev, component_instance.local_data[variable])
        """
        for i in range(0, 2):
            component_instance.modified_data[i] = morse.modifiers.gaussian.gaussian(self.dev, component_instance.modified_data[i])

        return component_instance.modified_data
        """
