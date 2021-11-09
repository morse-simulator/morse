import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSSubscriber, MOOSNotifier
from morse.core import blenderapi
import numpy as np
import capnp
import json
import sys
sys.path.append("/usr/local/etc/cortex")
cortex = None
try:
    import cortex_capnp as cortex
    CortexTextureType = cortex.TextureDescription.TextureType
except:
    print("\033[1;31m")
    print("Cortex could not be found!")
    print("\033[0m")
    ### Needed so that the function convert_texture_type_to_string_prefix can be defined when corex is undefined
    CortexTextureType = None
    
def convert_texture_type_to_string_prefix(texture_type: CortexTextureType)->str:
    if texture_type == CortexTextureType.rgbaTexture:
        return 'RGBA'
    raise RuntimeError('texture type not handled')

class objectServerReader(MOOSSubscriber):
    """ Read radar commands and update local data. """

    def initialize(self):
        
        ### If cortex is not installed and you do not have access to it then skip
        if cortex == None:
            return

        # Initialize the parent class
        MOOSSubscriber.initialize(self)

        # Register for scene query messages of type 'get' or 'get object'
        success = True
        success = self.register_message_to_queue('INVENTORY_REQUEST', 'moos_msg_queue', self.moos_msg_queue) and success
        success = self._comms.add_message_route_to_active_queue('moos_msg_queue', 'MESH_REQUEST') and self.register('MESH_REQUEST') and success
        success = self._comms.add_message_route_to_active_queue('moos_msg_queue', 'TEXTURE_REQUEST') and self.register('TEXTURE_REQUEST') and success
        if not success:
            logger.error("Failed to register messages to queue")
        # return success

    def moos_msg_queue(self, msg):
        if msg.key() == 'INVENTORY_REQUEST':
            self.data['inventory_requests'].put(int(msg.double()))
        elif msg.key() == 'MESH_REQUEST':
            self.data['mesh_requests'].put(msg.string())
        elif msg.key() == 'TEXTURE_REQUEST':
            self.data['texture_requests'].put(msg.binary_data())
        self._new_messages = False
        return True

class objectServerNotifier(MOOSNotifier):

    def initialize(self):
        MOOSNotifier.initialize(self)
        # Publish reset
        self.notify('OBJECT_SERVER_RESET', '')

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
        elif not self.data['mesh_responses'].empty():
            mesh_data = self.data['mesh_responses'].get()
            if isinstance(mesh_data, cortex.Mesh.Builder):
                self._comms.notify_binary('MESH', mesh_data.to_bytes())
            else:
                logger.error('Unknown object data type - cannot send data')
        else:
            for texture_type, texture_response_queue in self.data['texture_responses'].items():
                if not texture_response_queue.empty():
                    texture = texture_response_queue.get()
                    if isinstance(texture, cortex.Texture.Builder):
                        msg_variable = convert_texture_type_to_string_prefix(texture_type) + '_TEXTURE'
                        self._comms.notify_binary(msg_variable, texture.to_bytes())
                        logger.info('Sent ' + texture.identifier + ' on ' + msg_variable)
                    else:
                        logger.error('Unknown texture data type - cannot send data')
                    break # after one texture
            
            for texture_type, uvs_response_queue in self.data['uvs_responses'].items():
                if not uvs_response_queue.empty():
                    uvs = uvs_response_queue.get()
                    if isinstance(uvs, cortex.Uvs.Builder):
                        msg_variable = convert_texture_type_to_string_prefix(texture_type) + '_UVS'
                        self._comms.notify_binary(msg_variable, uvs.to_bytes())
                        logger.info('Sent ' + uvs.identifier + ' on ' + msg_variable)
                    else:
                        logger.error('Unknown uvs data type - cannot send data')
                    break # after one uvs
        
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
