# -*- coding: utf-8 -*-

import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
import morse.core.actuator
from morse.core.services import service
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
    
    add_property('_emit', True, 'emit', 'bool', "Set to False to initially disable the light")
    add_property('_distance', 10, 'distance', 'float', "Distance at which the light energy is halfed")
    add_property('_color', "(0,0,0)", 'color', 'string', "Light color")
    add_property('_size', pi/2, 'size', 'float', "Light spot size")
    add_property('_energy', 1.0, 'energy', 'float', "Light energy when On")

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        # get the light which is a child of the Empty object
        self.light = self.bge_object.children[0] 
        
        self.local_data['emit'] = self._emit
        self._last = not self.local_data['emit']

        logger.info('Component initialized')

    def default_action(self):
        """ Switch on/off the light. """
        # if no changes, return
        if self._last == self.local_data['emit']:
            return

        if self.local_data['emit']:
            self.light.energy = self._energy
        else:
            self.light.energy = 0.0

        # for each camera's scene: update the light
        for scene in blenderapi.get_scene_list():
            if scene.name not in ['S.MORSE_ENV', 'S.MORSE_LOGIC'] and \
                    self.light.name in scene.objects:
                scene.objects[self.light.name].energy = self.light.energy

        self._last = self.local_data['emit']

    @service
    def toggle(self, emit=None):
        """
        Toggle the light.

        :param emit: if emit is set to True/False, switch the light On/Off;
            if emit is not set, toggle the light (from On to Off and conversely)
        :return: Returns always True
        """
        if emit:
            self.local_data['emit'] = emit
        else:
            self.local_data['emit'] = not self.local_data['emit']
        return True
