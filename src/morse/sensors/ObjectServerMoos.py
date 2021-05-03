import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSSubscriber, MOOSNotifier
from morse.core import blenderapi
import numpy as np
import capnp
import json
import sys
sys.path.append("/usr/local/etc/cortex")
import cortex_capnp as cortex

class objectServerReader(MOOSSubscriber):
    """ Read radar commands and update local data. """

    def initialize(self):

        # Initialize the parent class
        MOOSSubscriber.initialize(self)

        # Register for scene query messages of type 'get' or 'get object'
        success = True
        success = self.register_message_to_queue('INVENTORY_REQUEST', 'moos_msg_queue', self.moos_msg_queue) and success
        success = self._comms.add_message_route_to_active_queue('moos_msg_queue', 'OBJECT_REQUEST') and self.register('OBJECT_REQUEST') and success
        if not success:
            logger.error("Failed to register messages to queue")
        return success

    def moos_msg_queue(self, msg):
        if msg.key() == 'INVENTORY_REQUEST':
            self.data['inventory_requests'].put(int(msg.double()))
        elif msg.key() == 'OBJECT_REQUEST':
            self.data['object_requests'].put(msg.binary_data())
        self._new_messages = False
        return True

class objectServerNotifier(MOOSNotifier):

    def default(self, ci = 'unused'):

        if not self.data['inventory_responses'].empty():
            variable = 'INVENTORY_FULL'
            inventory_response = self.data['inventory_responses'].get()
            uid = -1
            if isinstance(inventory_response, dict):
                self.notify(variable, json.dumps(inventory_response))
                uid = inventory_response["uid"]
            else:
                self._comms.notify_binary(variable, inventory_response.to_bytes())
                uid = inventory_response.uid
            logger.info("Sent INVENTORY_FULL on uid " + str(uid))
        elif not self.data['object_responses'].empty():
            object_data = self.data['object_responses'].get()
            if isinstance(object_data, cortex.Mesh.Builder):
                self._comms.notify_binary('MESH', object_data.to_bytes())
            else:
                logger.error('Unknown object data type - cannot send data')
        
        if self.data['inventory_updates'] is not None:
            variable = 'INVENTORY_UPDATE'
            inventory_update = self.data['inventory_updates']
            if isinstance(inventory_update, dict):
                self.notify(variable, json.dumps(inventory_update))
            else:
                self._comms.notify_binary(variable, inventory_update.to_bytes())
            self.data['inventory_updates'] = None

    def update_morse_data(self):
        logger.debug('objectServerNotifier.update_morse_data() called.')
