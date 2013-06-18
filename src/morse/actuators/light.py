# -*- coding: utf-8 -*-

import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
import morse.core.actuator
from morse.helpers.components import add_data, add_property
from math import pi

class Light(morse.core.actuator.Actuator):
    """
    This actuator is a simple On/Off light. Based on `SPOT
    <http://wiki.blender.org/index.php/Doc:2.6/Manual/Lighting/Lamps/Spot>`_ light.

    -  Emit in +X

    """
    _name = "Light"
    _short_desc = "A simple point light"

    add_data('emit', True, 'bool', 'On/Off light switch')
    
    add_property('_emit', True, 'emit', doc="Set to False to initially disable the light")
    add_property('_distance', 10, 'distance', doc="Distance at which the light energy is halfed")
    add_property('_color', "(0,0,0)", 'color', doc="Light color")
    add_property('_size', pi/2, 'size', doc="Light spot size")
    add_property('_energy', 1.0, 'energy', doc="Light energy when On")

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        # get the light which is a child of the Empty object
        self.light = self.bge_object.children[0] 
        
        self.local_data['emit'] = self._emit

        logger.info('Component initialized')

    def default_action(self):
        """ Switch on/off the light. """

        if self.local_data['emit']:
            self.light.energy = self._energy
        else:
            self.light.energy = 0.0

