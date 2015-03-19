import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware import AbstractDatastream
from morse.middleware.hla.message_buffer import MessageBufferReader

class CertiTestInput(AbstractDatastream):
    def initialize(self):
        self._amb = self.kwargs['__hla_node'].morse_ambassador
        self._hla_name = self.component_instance.robot_parent.name() 

        bille_handle = self._amb.object_handle('Bille')

        self.handle_x = self._amb.attribute_handle("PositionX", bille_handle)
        self.handle_y = self._amb.attribute_handle("PositionY", bille_handle)

        self._amb.suscribe_attributes(self._hla_name, bille_handle, [self.handle_x, self.handle_y])

    def finalize(self):
        pass

    def default(self, ci = 'unused'):
        attributes = self._amb.get_attributes(self._hla_name)
        if attributes and attributes[self.handle_x] and attributes[self.handle_y]:
            x = MessageBufferReader(attributes[self.handle_x]).read_double()
            y = MessageBufferReader(attributes[self.handle_y]).read_double()
            logger.info("%s pose %f %f" % (self._hla_name, x, y))
            self.data['x'] = x
            self.data['y'] = y
            self.data['z'] = 0.0

