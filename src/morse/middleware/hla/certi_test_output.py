import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.hla.message_buffer import MessageBufferWriter
from morse.middleware.hla.abstract_hla import AbstractHLAOutput


class CertiTestOutput(AbstractHLAOutput):
    def initialize(self):
        AbstractHLAOutput.initialize(self)

        boule_handle = self.amb.object_handle('Boule')

        self.handle_x = self.amb.attribute_handle("PositionX", boule_handle)
        self.handle_y = self.amb.attribute_handle("PositionY", boule_handle)

        self.amb._rtia.publishObjectClass(boule_handle, [self.handle_x, self.handle_y])
        self.register_object(boule_handle)

    def default(self, ci = 'unused'):
        to_send =  \
                {self.handle_x: MessageBufferWriter().write_double(self.data['x']),
                 self.handle_y: MessageBufferWriter().write_double(self.data['y'])}
        self.update_attribute(to_send)
