import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.hla.message_buffer import MessageBufferReader
from morse.middleware.hla.abstract_hla import AbstractHLAInput

class CertiTestInput(AbstractHLAInput):
    def initialize(self):
        AbstractHLAInput.initialize(self)

        bille_handle = self.amb.object_handle('Bille')

        self.handle_x = self.amb.attribute_handle("PositionX", bille_handle)
        self.handle_y = self.amb.attribute_handle("PositionY", bille_handle)

        self.suscribe_attributes(bille_handle, [self.handle_x, self.handle_y])

    def default(self, ci = 'unused'):
        attributes = self.get_attributes()

        if attributes and attributes[self.handle_x] and attributes[self.handle_y]:
            x = MessageBufferReader(attributes[self.handle_x]).read_double()
            y = MessageBufferReader(attributes[self.handle_y]).read_double()
            logger.info("%s pose %f %f" % (self.hla_name(), x, y))
            self.data['x'] = x
            self.data['y'] = y
            self.data['z'] = 0.0

