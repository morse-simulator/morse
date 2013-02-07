# -*- coding: utf-8 -*-

import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
import morse.core.actuator
from morse.helpers.components import add_data

class Light(morse.core.actuator.Actuator):
    """
    This actuator is a simple On/Off light. Based on `SPOT
    <http://wiki.blender.org/index.php/Doc:2.6/Manual/Lighting/Lamps/Spot>`_ light.

    Properties
    ----------

    -  Emit in +X
    -  Spot size = 90°
    -  Distance = 10m
    -  Energy: On = 1.0; Off = 0.0

    """
    _name = "Light"
    _short_desc = "A simple point light"

    add_data('emit', True, 'bool', 'On/Off light switch')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        self.local_data['emit'] = obj['emit']

        logger.info('Component initialized')

    def default_action(self):
        """ Apply (v, w) to the parent robot. """
        # get the Blender Logic Controller
        contr = blenderapi.controller()
        # get the Empty object parent of this Controller
        light_act = contr.owner
        # get the light which is a child of the Empty object
        light = light_act.children[0]

        # switch on/off the light
        if self.local_data['emit']:
            light.energy = 1.0
        else:
            light.energy = 0.0

