import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSSubscriber, MOOSNotifier
from math import inf

class FLSReader(MOOSSubscriber):
    """ Read FLS commands and update local data. """

    def initialize(self):

        # initialize the parent class
        MOOSSubscriber.initialize(self)
        
        # register for control variables from the database
        self.register_message_to_queue('FLS_STATUS','FLS_queue', self.on_FLS_msgs)
        
    def on_FLS_msgs(self, msg):
        if (msg.key() == 'FLS_STATUS') and (msg.is_string()):
            self.data['status'] = msg.string()

        self._new_messages = True
        return True

class FLSNotifier(MOOSNotifier):
    """ Notify FLS """

    def default(self, ci = 'unused'):

        logger.debug('FLSNotifier is publishing!')
        if self.data['range'] != inf:
            self.notify('FLS_RANGE', self.data['range'])
            self.data['range'] = inf

        if len(self.data['feature']):
            self.notify('TRACKED_FEATURE', self.data['feature'])
            self.data['feature'] = ''
        
