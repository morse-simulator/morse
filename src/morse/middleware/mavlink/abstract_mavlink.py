from morse.middleware import AbstractDatastream
import logging; logger = logging.getLogger("morse." + __name__)
from pymavlink.mavutil import mavlink_connection 

class MavlinkSensor(AbstractDatastream):
    def initialize(self):
        self._mavlink_client = None
        self._boot_time = 0
        self._msg = None

    def setup(self, mavlink_client, boot_time):
        self._mavlink_client = mavlink_client
        self._boot_time = boot_time

    """
    common messages requires time in ms since boot, so provide an helper
    function to compute that
    """
    def time_since_boot(self):
        return int((self.data['timestamp'] - self._boot_time) * 1000)

    """
    The make_msg is a method which must be overrided The goal of the
    method is to set with something sensible the self._msg field
    """
    def make_msg(self):
        pass

    def default(self, ci = 'unused'):
        self.make_msg()
        self._mavlink_client.send(self._msg)

class MavlinkActuator(AbstractDatastream):
    def initialize(self):
        self._conn = mavlink_connection(self.kwargs['device'])
        self._msg = None


    def process_msg(self):
        """
        The process_msg must be overriden. It should convert mavlink
        format into the internal Morse format. The last mavlink message
        is stored in self._msg.
        """
        pass

    def default(self, ci = 'unused'):
        self._msg = self._conn.recv_msg()
        if not self._msg:
            return False
        logger.debug('Received %s' % self._msg)
        self.process_msg()
        return True
