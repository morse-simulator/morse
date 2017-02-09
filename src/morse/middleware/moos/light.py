import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSSubscriber

class LightReader(MOOSSubscriber):
    """ Read light commands. """

    def initialize(self):
        MOOSSubscriber.initialize(self)
        self.register_message_to_queue('MORSE_LIGHT',
                    'light_queue', self.on_light_msg)

    def on_light_msg(self, msg):
        if msg.key() == 'MORSE_LIGHT':
            self.data['emit'] = (msg.string().lower()=="true")
