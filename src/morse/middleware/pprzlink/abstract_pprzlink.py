import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware import AbstractDatastream

from pprzlink.ivy import IvyMessagesInterface
from pprzlink.message import PprzMessage 

class classproperty(object):
    def __init__(self, fget):
        self.fget = fget
    def __get__(self, owner_self, owner_cls):
        return self.fget(owner_cls)

class PprzlinkDatastream(AbstractDatastream):
    def initialize(self):
        name = 'pprzlink'
        self._interface = IvyMessagesInterface("pprzlink_morse")

    @classproperty
    def _type_url(cls):
        return "http://docs.paparazziuav.org/latest/paparazzi_messages.html#" + cls._type_name

    def finalize(self):
        if self._interface is not None:
            self._interface.shutdown()
            self._interface = None

class PprzlinkSensor(PprzlinkDatastream):
    def initialize(self):
        self._msg = None

    def make_msg(self):
        """
        The make_msg is a method which must be overrided The goal of the
        method is to set with something sensible the self._msg field
        """
        pass

    def default(self, ci = 'unused'):
        self.make_msg()
        self._interface.send(self._msg)

class PprzlinkActuator(PprzlinkDatastream):
    def initialize(self):
        PprzlinkDatastream.initialize(self)
        self._msg = None

        """ Bind to specific message or all messages otherwise """
        if 'msg_name' in self.kwargs:
            self.msg_name = self.kwargs['msg_name']
        else:
            self.msg_name = self._type_name
        if 'ac_id' in self.kwargs:
            self.ac_id = self.kwargs['ac_id']
        else:
            self.ac_id = '[0-9]+'

        def msg_callback(ac_id, msg):
            if self._msg is None:
                self._msg = msg

        self._interface.subscribe(msg_callback, regex=('(^'+str(self.ac_id)+' '+self.msg_name+' .*)'))

    def process_msg(self):
        """
        The process_msg must be overriden. It should convert mavlink
        format into the internal Morse format. The last pprzlink message
        is stored in self._msg.
        """
        pass

    def default(self, ci = 'unused'):
        # If a new message has been received
        if self._msg:
            # Update local_data
            self.process_msg()
            # Free message for new reception
            self._msg = None
            # Tell MORSE that we can apply modifiers
            return True

        return False

