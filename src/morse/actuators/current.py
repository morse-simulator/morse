import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.actuator

from morse.core.services import service, async_service, interruptible
from morse.core import status
from morse.helpers.components import add_data, add_property
from math import radians, degrees, pi
from morse.core import blenderapi
from morse.core.mathutils import *
import random

__author__     = "David Battle"
__copyright__  = "Copyright 2019, Mission Systems Pty Ltd"
__license__    = "BSD"
__version__    = "1.0.0"
__maintainer__ = "David Battle"
__email__      = "david.battle@missionsystems.com.au"
__status__     = "Production"

class Current(morse.core.actuator.Actuator):
    """Write here the general documentation of your actuator.
    It will appear in the generated online documentation.
    """
    _name = "Current"
    _short_desc = "Simple current simulator with turbulence"

    add_property('_current_speed', 0, 'Current_Speed', 'float', 'Current speed in m/s')
    add_property('_current_dir',   0,   'Current_Dir', 'float', 'Bearing from which current is coming in deg')
    add_property('_turbulence',    0,    'Turbulence', 'float', 'Current std deviation in m/s')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        # Get every bge object in the scene
        self.bge_objs = blenderapi.scene().objects

        # Current object (from environment)
        self.current = self.bge_object

        # Initialise current vector
        self.current['vec'] = Vector([0,0,0])

        current_dir = radians(self._current_dir - 90)
        self.current_vec_world = Matrix.Rotation(current_dir, 3, 'Z').col[0]

        logger.info('Current speed is %.1f m/s', self._current_speed)
        logger.info('Current direction is from %.1f degrees', self._current_dir)
        logger.info('Current variance is %.1f m/s', self._turbulence)
        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):

        # Constant current with gusts
        current_speed = random.gauss(self._current_speed, self._turbulence)

        # Instantaneous current vector
        current_world = current_speed * self.current_vec_world

        # Store current property
        self.current['vec'] = current_world
