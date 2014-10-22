import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware import AbstractDatastream
from hla.message_buffer import MessageBufferWriter


class CertiTestOutput(AbstractDatastream):
    def initialize(self):
        self._amb = self.kwargs['__hla_node'].morse_ambassador

        boule_handle = self._amb.object_handle('Boule')

        self.handle_x = self._amb.attribute_handle("PositionX", boule_handle)
        self.handle_y = self._amb.attribute_handle("PositionY", boule_handle)

        self._amb._rtia.publishObjectClass(boule_handle, [self.handle_x, self.handle_y])
        self.boule = self._amb.register_object(boule_handle, self.component_instance.robot_parent.name())

    def default(self, ci = 'unused'):
        to_send =  \
                {self.handle_x: MessageBufferWriter().write_double(self.data['x']),
                 self.handle_y: MessageBufferWriter().write_double(self.data['y'])}
        self._amb.update_attribute(self.boule, to_send)
