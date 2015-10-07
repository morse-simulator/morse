import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.mavlink.abstract_mavlink import MavlinkSensor
from pymavlink.dialects.v10 import common as mavlink

class OdometrySensor(MavlinkSensor):
    _type_name = "LOCAL_POSITION_NED"

    def make_msg(self):
        # Expects the coordinate in aeronautical frame, so doing ENU ->
        # NED conversion
        self._msg = mavlink.MAVLink_local_position_ned_message(
                self.time_since_boot(),
                self.data['x'],
                - self.data['y'],
                - self.data['z'],
                self.data['vx'],
                - self.data['vy'],
                - self.data['vz']
        )
