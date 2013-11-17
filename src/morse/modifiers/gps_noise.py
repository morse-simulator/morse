import logging; logger = logging.getLogger("morse." + __name__)
from random import gauss, random
from math import radians, degrees, cos

from morse.helpers.components import add_property
from morse.modifiers.abstract_modifier import AbstractModifier

import morse.helpers.geo_math as geo_math

class GPSNoiseModifier(AbstractModifier):
    """
    This modifier allows to simulate Gaussian noise and constant bias for the compass.
    """

    _name = "GPSNoise"

    add_property('chance_of_fix', 1.0, 'chance_of_fix', 'float', 'Probability that the GPS has a fix, 1.0 means always has fix.')
    add_property('latlon_stddev', 0, 'latlon_stddev', 'float', 'Standard deviation of GPS Lat/Lon in meters.')
    add_property('alt_stddev', 0, 'alt_stddev', 'float', 'Standard deviation of the GPS Altitude in meters.')
    add_property('heading_stddev', 0, 'heading_stddev', 'float', 'Standard deviation of the GPS heading in degrees.')
    add_property('speed_stddev', 0, 'speed_stddev', 'float', 'Standard deviation of the GPS speed in meters-per-second.')

    def initialize(self):
        self.chance_of_fix = self.parameter('chance_of_fix', default=1)
        self.latlon_stddev = self.parameter('latlon_stddev', default=0)
        self.alt_stddev = self.parameter('alt_stddev', default=0)
        self.heading_stddev = self.parameter('heading_stddev', default=0)
        self.speed_stddev = self.parameter('speed_stddev', default=0)
        logger.info('GPS Noise chance of fix:%s, stddevs latlon:%0.2f, alt:%0.2f, heading:%0.2f, speed:%0.2f',
                    str(self.chance_of_fix), self.latlon_stddev, self.alt_stddev, self.heading_stddev, self.speed_stddev)  

    def modify(self):
        self.data['has_fix'] = random() < self.chance_of_fix
        if self.data['has_fix']:
            self.data['lat'], self.data['lon'] = geo_math.point_from_distance_and_heading(
                        self.data['lat'], self.data['lon'], gauss(0, self.latlon_stddev), 360.0*random())
            self.data['alt'] += gauss(0, self.alt_stddev)
            self.data['speed'] += gauss(0, self.speed_stddev)
            self.data['heading'] += gauss(0, self.heading_stddev)
        else:
            self.data['lat'] = 0
            self.data['lon'] = 0
            self.data['alt'] = 0
            self.data['speed'] = 0
            self.data['heading'] = 0



