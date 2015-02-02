from morse.middleware import AbstractDatastream

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
