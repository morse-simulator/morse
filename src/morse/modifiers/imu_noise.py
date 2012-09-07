import logging; logger = logging.getLogger("morse." + __name__)
import random

from morse.core.modifier import MorseModifierClass

class MorseIMUNoiseClass(MorseModifierClass):

    _gyro_std_dev = 0.05
    _accel_std_dev = 0.5

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
            logger.warning("Unknown function name for IMU Noise modifier. Check component_config.py file.")

        # Extract the Modifier parameters from the dictionary
        try:
            self._gyro_std_dev = mod_data[2].get("gyro_std", 0.05)
            self._accel_std_dev = mod_data[2].get("accel_std", 0.5)
        except:
            pass

        logger.info("Adding noise to IMU with standard deviations: gyro %.4f, accel %.4f", \
                    self._gyro_std_dev, self._accel_std_dev)


    def noisify(self, component_instance):
        for i in range(0, 3):
            component_instance.local_data['angular_velocity'][i] = \
                random.gauss(component_instance.local_data['angular_velocity'][i], self._gyro_std_dev)
            component_instance.local_data['linear_acceleration'][i] = \
                random.gauss(component_instance.local_data['linear_acceleration'][i], self._accel_std_dev)

