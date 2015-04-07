import logging
logger = logging.getLogger("morse." + __name__)
from morse.core.sensor import Sensor
from morse.core import blenderapi
from morse.helpers.components import add_data, add_property

from math import pow

MOLAR_MASS = 0.0289644  # kg / mol
GAS_CONSTANT = 8.31447 # J/(mol•K)
TEMPERATURE_LAPSE_RATE = 0.0065 # K/m
SEA_LEVEL_TEMP = 288.15 # K:

class Barometer(Sensor):
    """
    Sensor to compute the atmopsheric pressure, using the ISA model:

    - https://en.wikipedia.org/wiki/International_Standard_Atmosphere
    - http://en.wikipedia.org/wiki/Atmospheric_pressure

    The current implementation is only correct in the Troposphere, i.e.
    for an altitude less than 11000m
    """
    _name = "Barometer"
    _short_desc = "Mesure the atmospheric pressure"

    add_data('pressure', 0.0, "float", 'Pressure in Pa')
    add_property('_ref_p', 101325, "ReferencePressure", "float", 
                 "Reference pressue, in Pascal. By default, the standard \
                  pressure at the sea level")

    def __init__(self, obj, parent=None):
        """ Constructor method.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        Sensor.__init__(self, obj, parent)

        self._inv_exp = (- blenderapi.gravity()[2] * MOLAR_MASS) / \
                        (GAS_CONSTANT * TEMPERATURE_LAPSE_RATE)
        self._ref_z = self.position_3d.z

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):
        dz = self.position_3d.z - self._ref_z
        tmp = 1 - (TEMPERATURE_LAPSE_RATE * dz  / SEA_LEVEL_TEMP)
        self.local_data['pressure'] = self._ref_p * pow(tmp, self._inv_exp)
