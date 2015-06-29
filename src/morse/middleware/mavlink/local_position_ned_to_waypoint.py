import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.mavlink.abstract_mavlink import MavlinkActuator

class WaypointActuator(MavlinkActuator):
    def process_msg(self):
        self.data['x'] = self._msg.x
        self.data['y'] = -self._msg.y
        self.data['z'] = -self._msg.z
        self.data['tolerance'] = 0.5 # XXX
        self.data['yaw'] = 0.0 # XXX
