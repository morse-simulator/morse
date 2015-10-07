import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.mavlink.abstract_mavlink import MavlinkSensor
from pymavlink.dialects.v10 import common as mavlink

class AttitudeSensor(MavlinkSensor):
    _type_name = "ATTITUDE"

    def make_msg(self):
        # Expects the coordinate in aeronautical frame
        self._msg = mavlink.MAVLink_attitude_message(
                self.time_since_boot(),
                self.data['rotation'][0],
                - self.data['rotation'][1],
                - self.data['rotation'][2],
                self.data['angular_velocity'][0],
                - self.data['angular_velocity'][1],
                - self.data['angular_velocity'][2]
        )
