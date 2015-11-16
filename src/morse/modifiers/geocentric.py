import logging; logger = logging.getLogger("morse." + __name__)

from morse.modifiers.abstract_modifier import AbstractModifier
from morse.helpers.coordinates import CoordinateConverter
from math import degrees, radians
import numpy

class Geocentricmodifier(AbstractModifier):
    """ 
    This modifier converts the coordinates from MORSE simulator (in LTP)
    into Geocentric coordinates.

    To work properly, you need to configure the following variables at
    the environment level:
        - **longitude** in degrees (double) of Blender origin
        - **latitude** in degrees (double) of Blender origin
        - **altitude** in m  of the Blender origin
    The Geocentric modifier provides as modifiers:
    
    * :py:class:`morse.modifiers.ecef.CoordinatesToGeocentric`
    * :py:class:`morse.modifiers.ecef.CoordinatesFromGeocentric`
    """
    
    _name = "Geocentric"

    def initialize(self):
        self.converter = CoordinateConverter.instance()

class CoordinatesToGeocentric(Geocentricmodifier):
    """ Converts from Blender coordinates to Geocentric coordinates.
    """
    def modify(self):
        try:
            xe = numpy.matrix(
                    [
                    self.data['x'],
                    self.data['y'],
                    self.data['z']
                    ])
            xt = self.converter.ecef_to_geocentric(
                    self.converter.ltp_to_ecef(
                    self.converter.blender_to_ltp(xe)))

            logger.debug("%s => %s" % (xe, xt))

            self.data['x'] = degrees(xt[0, 0])
            self.data['y'] = degrees(xt[0, 1])
            self.data['z'] = xt[0, 2]
        except KeyError as detail:
            self.key_error(detail)

class CoordinatesFromGeocentric(Geocentricmodifier):
    """ Converts from Geocentric coordinates to Blender coordinates.
    """
    def modify(self):
        try:
            logger.info(self.data)
            xe = numpy.matrix(
                    [
                    radians(self.data['x']),
                    radians(self.data['y']),
                    self.data['z']
                    ])
            xt = self.converter.blender_to_ltp(
                    self.converter.ecef_to_ltp(
                    self.converter.geocentric_to_ecef(xe)))

            logger.debug("%s => %s" % (xe, xt))

            self.data['x'] = xt[0, 0]
            self.data['y'] = xt[0, 1]
            self.data['z'] = xt[0, 2]
        except KeyError as detail:
            self.key_error(detail)
