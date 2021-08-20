import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSSubscriber, MOOSNotifier
from morse.core import blenderapi
import numpy as np

class objectServerReaderDeprecated(MOOSSubscriber):
    """ Read radar commands and update local data. """

    def initialize(self):

        # Initialize the parent class
        MOOSSubscriber.initialize(self)

        # Register for scene query messages of type 'get' or 'get object'
        return self.register_message_to_queue('SCENE_QUERY','scene_query_queue', self.on_query_msgs)

    def on_query_msgs(self, msg):

        if (msg.key() == 'SCENE_QUERY') and (msg.is_string()):

            # Store new queries received from MOOS.
            # We implement a queue so we don't lose any

            # We skip "get" inventory calls from piling up by not processing 
            # them if there are other messages in the queue
            if msg.string() != "get" or len(self.data['scene_query']) == 0:
                self.data['scene_query'].append(msg.string())

        self._new_messages = True

        return True

class objectServerNotifierDeprecated(MOOSNotifier):

    def default(self, ci = 'unused'):

        #logger.debug('objectServerNotifier is publishing!')

        # Time stamp for outgoing data
        ts = self.data['timestamp']

        if self.data['scene_data']:
            # Send the JSON data to MOOS
            self.notify('SCENE_DATA', self.data['scene_data'], ts)
            # Cancel message for next cycle
            self.data['scene_data'] = ''

        if self.data['object_data_binary']:
            # notify_binary() is not exposed through the MOOSNotifier base class
            # so we go via the _comms object
            self._comms.notify_binary('OBJECT_DATA_BINARY', self.data['object_data_binary'], ts)
            # Cancel message for next cycle
            self.data['object_data_binary'] = ''

        if self.data['test_data']:
            # Send the test data to MOOS
            # notify_binary() is not exposed through the MOOSNotifier base class
            # so we go via the _comms object
            self._comms.notify_binary('TEST_DATA', self.data['test_data'], ts)
            # Cancel message for next cycle
            self.data['test_data'] = ''

    def update_morse_data(self):
        logger.debug('objectServerNotifier.update_morse_data() called.')
