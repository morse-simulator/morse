import logging; logger = logging.getLogger("morse." + __name__)

from morse.modifiers.abstract_modifier import AbstractModifier

class FeetModifier(AbstractModifier):
    """ 
    This modifier converts datas from imperial units to metrics units and vice-versa.

    The Feet modifier provides as modifiers:
    
    * :py:class:`morse.modifiers.feet.MeterToFeet`
    * :py:class:`morse.modifiers.feet.FeetToMeter`

    """
    
    _name = "feet"

    def initialize(self):
        self._coeff = 1.0
    
    def modify(self):
        for key in {'x', 'y', 'z'}:
            if key in self.data:
                self.data[key] *= self._coeff

class MeterToFeet(FeetModifier):
    """ Converts Meter (Morse) to Feet
    """
    def initialize(self):
        self._coeff = 3.2808399

class FeetToMeter(FeetModifier):
    """ Converts Feet to Meter
    """
    def initialize(self):
        self._coeff = 1/3.2808399
