import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSSubscriber, MOOSNotifier
from morse.core import blenderapi
import json

class CameraSimNotifier(MOOSNotifier):
    """ Notify camerasim """

    def default(self, ci = 'unused'):
        launch_trigger = self.data['launch_trigger']
        msg_name = self.data['camera_name'] + '_TRIGGER'
        if isinstance(launch_trigger, dict):
            self.notify(msg_name, json.dumps(launch_trigger))
        else:
            self._comms.notify_binary(msg_name, launch_trigger.to_bytes())

    def update_morse_data(self):
        logger.debug('CameraSimNotifier.update_morse_data() called.')
