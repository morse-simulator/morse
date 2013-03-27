import logging; logger = logging.getLogger("morse." + __name__)

from morse.helpers.components import add_property
from morse.modifiers.abstract_modifier import AbstractModifier

class UTMModifier(AbstractModifier):
    """ Convert between Blender and UTM coordinates. """
    
    def initialize(self):
        """ Initialize the UTM coordinates reference. """
        self._x_offset = float(self.parameter('x_offset', prop='UTMXOffset', default=0))
        self._y_offset = float(self.parameter('y_offset', prop='UTMYOffset', default=0))
        self._z_offset = float(self.parameter('z_offset', prop='UTMZOffset', default=0))
        logger.info("UTM reference point is (%s,%s,%s)" 
                    % (self._x_offset, self._y_offset, self._z_offset))

class CoordinatesToUTM(UTMModifier):
    def modify(self):
        try:
            self.data['x'] += self._x_offset
            self.data['y'] += self._y_offset
            self.data['z'] += self._z_offset
        except KeyError as detail:
            self.key_error(detail)

class CoordinatesFromUTM(UTMModifier):
    def modify(self):
        try:
            self.data['x'] -= self._x_offset
            self.data['y'] -= self._y_offset
            self.data['z'] -= self._z_offset
        except KeyError as detail:
            self.key_error(detail)
