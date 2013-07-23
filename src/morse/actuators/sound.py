import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
from morse.core.actuator import Actuator
from morse.helpers.components import add_data

class Sound(Actuator):
    """
    This actuator is a simple On/Off sound. Based on `Sound
    <http://wiki.blender.org/index.php/Doc:2.6/Manual/Game_Engine/Logic/Actuators/Sound>`_
    actuator.
    """
    _name = "Sound"
    _short_desc = "A simple sound actuator"

    add_data('mode', "stop", 'string', "mode, enum in ['play','pause','stop']")

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        Actuator.__init__(self, obj, parent)

        self.local_data['mode'] = obj['mode']
        self._last_mode = None

        logger.info('Component initialized')

    def default_action(self):
        """ Apply ``play`` to this actuator. """
        # get the Blender Logic Controller
        contr = blenderapi.controller()
        # http://www.blender.org/documentation/blender_python_api_2_65_release/bge.types.html#bge.types.KX_SoundActuator
        if self.local_data['mode'] == self._last_mode:
            return
        act = contr.actuators[-1]
        contr.activate(act) # enables 3D effect (!)
        if self.local_data['mode'] == 'play':
            act.startSound()
        elif self.local_data['mode'] == 'pause':
            act.pauseSound()
        elif self.local_data['mode'] == 'stop':
            act.stopSound()
        # new last mode
        self._last_mode = self.local_data['mode']

