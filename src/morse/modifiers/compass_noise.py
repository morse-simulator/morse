import logging; logger = logging.getLogger("morse." + __name__)
from random import gauss

from morse.helpers.components import add_property
from morse.modifiers.abstract_modifier import AbstractModifier

class CompassNoiseModifier(AbstractModifier):
    """
    This modifier allows to simulate Gaussian noise and constant bias for the compass.
    """

    _name = "CompassNoise"

    add_property('heading_stddev', 0, 'heading_stddev', 'float', 'Standard deviation of the heading in degrees.')
    add_property('heading_bias', 0, 'heading_bias', 'float', 'Bias of the compass in degrees.')
        
    def initialize(self):
        self.heading_stddev = self.parameter('heading_stddev', default=0)
        self.heading_bias = self.parameter('heading_bias', default=0)
        logger.info("Compass Noise stddev %0.2f bias %0.2f" % (self.heading_stddev, self.heading_bias))

    def modify(self):
        self.data['heading'] += gauss(self.heading_bias, self.heading_stddev)


