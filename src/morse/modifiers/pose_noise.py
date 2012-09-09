import logging; logger = logging.getLogger("morse." + __name__)
import random
from math import radians, degrees, cos
import mathutils

from morse.core.modifier import MorseModifierClass

class MorsePoseNoiseClass(MorseModifierClass):

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
            logger.warning("Unknown function name for Pose Noise modifier. Check component_config.py file.")

        self._pos_std_dev = 0.05
        self._rot_std_dev = radians(5)
        # Extract the Modifier parameters from the dictionary if it is given
        try:
            self._pos_std_dev = mod_data[2].get("pos_std", self._pos_std_dev)
            self._rot_std_dev = mod_data[2].get("rot_std", self._rot_std_dev)
        except:
            pass

        logger.info("Adding noise to Pose with standard deviations: position %.4f, rotation %.4f deg", \
                    self._pos_std_dev, degrees(self._rot_std_dev))


    def noisify(self, component_instance):
        # add noise on position
        for variable in ['x', 'y', 'z']:
            component_instance.local_data[variable] = \
                random.gauss(component_instance.local_data[variable], self._pos_std_dev)

        # generate a gaussian noise rotation vector
        rot_vec = mathutils.Vector((0.0, 0.0, 0.0))
        for i in range(0, 3):
            rot_vec[i] = random.gauss(rot_vec[i], self._rot_std_dev)
        # convert rotation vector to a quaternion representing the random rotation
        angle = rot_vec.length
        if angle > 0:
            axis = rot_vec / angle
            noise_quat = mathutils.Quaternion(axis, angle)
        else:
            noise_quat = mathutils.Quaternion()
            noise_quat.identity()
        try:
            component_instance.local_data['orientation'] = (noise_quat * component_instance.local_data['orientation']).normalized()
        except:
            # for eulers this is a bit crude, maybe should use the noise_quat here as well...
            for variable in ['roll', 'pitch', 'yaw']:
                component_instance.local_data[variable] = \
                    random.gauss(component_instance.local_data[variable], self._rot_std_dev)

