import logging; logger = logging.getLogger("morse." + __name__)

from morse.core.modifier import MorseModifierClass
from math import cos, sin

class MorseOdometryNoiseClass(MorseModifierClass):
    def __init__(self):
        super(self.__class__, self).__init__()
        self._factor = 1.05
        self._gyro_drift = 0.00000005

        self._drift_x = 0.0
        self._drift_y = 0.0
        self._drift_yaw = 0.0

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
            logger.warning("Unknown function name for Odometry Noise modifier."
                            "Check component_config.py file.")

        # Extract the Modifier parameters
        try:
            self._factor = mod_data[2].get("factor", 1.05)
            self._gyro_drift = mod_data[2].get("gyro_drift", 0.0)
        except:
            pass

    def noisify(self, component_instance):
        
        # Basic 2D odometry implementation dx = dS * sin(yaw) and
        #                                  dy = dS * cos(yaw)
        # If we have some error on  dS and yaw, we have
        # dx      = factor * dS * sin(yaw + drift_yaw)
        #         = factor * dS * sin(yaw) * cos(drift_yaw) +
        #           factor * dS * cos(yaw)  * sin(drift_yaw)
        #         = factor * ( dx * cos(drift_yaw) +  dy * sin(drift_yaw))
        # Same thing to compute dy

        data = component_instance.local_data

        self._drift_yaw += self._gyro_drift

        dx = self._factor * ( data['dx'] * cos(self._drift_yaw) +
                              data['dy'] * sin(self._drift_yaw))
        dy = self._factor * ( data['dy'] * cos(self._drift_yaw) -
                              data['dx'] * sin(self._drift_yaw))

        self._drift_x +=  dx - data['dx']
        self._drift_y +=  dy - data['dy']

        data['dS'] *= self._factor
        data['dx'] = dx
        data['dy'] = dy
        data['dyaw'] += self._gyro_drift

        data['x'] += self._drift_x
        data['y'] += self._drift_y
        data['yaw'] += self._drift_yaw

        freq = component_instance.frequency

        data['vx'] = data['dx'] / freq
        data['vy'] = data['dy'] / freq
        data['wz'] = data['dyaw'] / freq


