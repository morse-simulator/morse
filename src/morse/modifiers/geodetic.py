import logging; logger = logging.getLogger("morse." + __name__)

from morse.modifiers.abstract_modifier import AbstractModifier
from morse.helpers.coordinates import CoordinateConverter
from math import degrees, radians
from morse.core import mathutils

class Geodeticmodifier(AbstractModifier):
    """ 
    This modifier converts the coordinates from MORSE simulator (in LTP)
    into Geodetic coordinates.

    To work properly, you need to configure the following variables at
    the environment level:
        - **longitude** in degrees (double) of Blender origin
        - **latitude** in degrees (double) of Blender origin
        - **altitude** in m  of the Blender origin
    The Geodetic modifier provides as modifiers:
    
    * :py:class:`morse.modifiers.ecef.CoordinatesToGeodetic`
    * :py:class:`morse.modifiers.ecef.CoordinatesFromGeodetic`
    """
    
    _name = "Geodetic"

    def initialize(self):
        self.converter = CoordinateConverter.instance()

class CoordinatesToGeodetic(Geodeticmodifier):
    """ Converts from Blender coordinates to Geodetic coordinates.
    """
    def modify(self):
        try:
            xe = mathutils.Vector(
                    (
                    self.data['x'],
                    self.data['y'],
                    self.data['z']
                    ))
            xt = self.converter.ltp_to_geodetic(xe)

            self.data['x'] = degrees(xt[0])
            self.data['y'] = degrees(xt[1])
            self.data['z'] = xt[2]
        except KeyError as detail:
            self.key_error(detail)

class CoordinatesFromGeodetic(Geodeticmodifier):
    """ Converts from Geodetic coordinates to Blender coordinates.
    """
    def modify(self):
        try:
            xe = mathutils.Vector(
                    (
                    radians(self.data['x']),
                    radians(self.data['y']),
                    self.data['z']
                    ))
            xt = self.converter.geodetic_to_ltp(xe)

            self.data['x'] = xt[0]
            self.data['y'] = xt[1]
            self.data['z'] = xt[2]
        except KeyError as detail:
            self.key_error(detail)
