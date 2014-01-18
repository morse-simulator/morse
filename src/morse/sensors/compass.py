import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.helpers.components import add_data, add_property

from math import degrees
from random import gauss

class Compass(morse.core.sensor.Sensor):
    """ 
    This sensor emulates a magnetic tilt-compensated compass.
    """
    _name = "Compass"

    add_data('heading', 0.0, "float", 'heading of the sensor in degrees.')

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

        logger.info('Compass component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):
        """ Get the heading from the yaw of the blender object. """
        self.local_data['heading'] = -degrees(self.position_3d.yaw)

