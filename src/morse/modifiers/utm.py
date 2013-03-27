import logging; logger = logging.getLogger("morse." + __name__)

from morse.helpers.components import add_property
from morse.modifiers.abstract_modifier import AbstractModifier

class UTMModifier(AbstractModifier):
    """ Convert between Blender and UTM coordinates. """
    
    def initialize(self):
        """ Initialize the UTM coordinates reference. """
        self._x_offset = float(self.parameter('x_offset', prop='UTMXOffset'))
        self._y_offset = float(self.parameter('y_offset', prop='UTMYOffset'))
        self._z_offset = float(self.parameter('z_offset', prop='UTMZOffset'))
        logger.info("UTM reference point is (%s,%s,%s)" 
                    % (self._x_offset, self._y_offset, self._z_offset))

class CoordinatesToUTM(UTMModifier):
    def modify(self):
        try:
            self.data['x'] += self._x_offset
            self.data['y'] += self._y_offset
            self.data['z'] += self._z_offset
        except KeyError as detail:
            logger.warning("Unable to use %s on component %s. It does not have data %s." 
                           % (self.__class__.__name__, self.component_name, detail))

class CoordinatesFromUTM(UTMModifier):
    def modify(self):
        try:
            self.data['x'] -= self._x_offset
            self.data['y'] -= self._y_offset
            self.data['z'] -= self._z_offset
        except KeyError as detail:
            logger.warning("Unable to use %s on component %s. It does not have data %s." 
                           % (self.__class__.__name__, self.component_name, detail))
