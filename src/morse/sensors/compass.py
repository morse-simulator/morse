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

    add_property('heading_stddev', 0, 'heading_stddev', 'float', 'Standard deviation of the heading in degrees.')
    add_property('heading_bias', 0, 'heading_bias', 'float', 'Bias of the compass in degrees.')

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        logger.info('Compass component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):
        """ Get the heading from the yaw of the blender object. """
        self.local_data['heading'] = -degrees(self.position_3d.yaw) + gauss(self.heading_bias, self.heading_stddev)

