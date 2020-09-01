import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.actuator

from morse.core.services import service, async_service, interruptible
from math import sin, pi
from morse.core import status
from morse.helpers.components import add_data, add_property
from morse.core import blenderapi
from morse.core.mathutils import *

class Sealevel(morse.core.actuator.Actuator):
    """Write here the general documentation of your actuator.
    It will appear in the generated online documentation.
    """
    _name = "Sealevel"
    _short_desc = "Varies the height of the sealevel in a semi-realistic manner"

    # define here the data fields required by your actuator
    # format is: field name, initial value, type, description
    add_property('amplitude', 0.5, 'Amplitude', 'float', 'Amplitude of sealevel height variation')
    add_property('period',    10,  'Period',    'float', 'Period of sealevel height variation')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        self.scene = morse.core.blenderapi.scene()

        # Get every bge object in the scene
        objs = blenderapi.scene().objects

        # Get the target object
        self.water = objs['water']
        self.phase = 0

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):

        # Game loop frequency
        delta_t = 1/self.frequency
        if delta_t == 0:
            return # Not ready yet!

        self.phase += 2 * pi / self.period * delta_t

        self.water.worldPosition.z = self.amplitude * sin(self.phase)
