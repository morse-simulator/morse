import logging; logger = logging.getLogger("morse." + __name__)

from morse.modifiers.abstract_modifier import AbstractModifier
from morse.helpers.coordinates import CoordinateConverter
import numpy

class ECEFmodifier(AbstractModifier):
    """ 
    This modifier converts the coordinates from MORSE simulator (in LTP)
    into ECEF-r coordinates.

    To work properly, you need to configure the following variables at
    the environment level:
        - **longitude** in degrees (double) of Blender origin
        - **latitude** in degrees (double) of Blender origin
        - **altitude** in m  of the Blender origin

    The ECEF modifier provides as modifiers:
    
    * :py:class:`morse.modifiers.ecef.CoordinatesToECEF`
    * :py:class:`morse.modifiers.ecef.CoordinatesFromECEF`
    """
    _name = "ECEF"

    def initialize(self):
        self.converter = CoordinateConverter.instance()

    def modify(self):
        try:
            xe = numpy.matrix(
                    [
                    self.data['x'],
                    self.data['y'],
                    self.data['z']
                    ])
            xt = self.method(xe)
            logger.info("%s => %s" % (xe, xt))

            self.data['x'] = xt[0, 0]
            self.data['y'] = xt[0, 1]
            self.data['z'] = xt[0, 2]
        except KeyError as detail:
            self.key_error(detail)

class CoordinatesToECEF(ECEFmodifier):
    """ Converts from Blender coordinates to ECEF coordinates.
    """
    def initialize(self):
        ECEFmodifier.initialize(self)

    def method(self, xt):
        return self.converter.ltp_to_ecef(self.converter.blender_to_ltp(xt))

class CoordinatesFromECEF(ECEFmodifier):
    """ Converts from UTM coordinates to Blender coordinates.
    """
    def initialize(self):
        ECEFmodifier.initialize(self)

    def method(self, xt):
        return self.converter.blender_to_ltp(self.converter.ecef_to_ltp(xt))
