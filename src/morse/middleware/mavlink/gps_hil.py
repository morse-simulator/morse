import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.mavlink.abstract_mavlink import MavlinkSensor
from pymavlink.dialects.v10 import common as mavlink
from math import sqrt

class GPSHILSensor(MavlinkSensor):
    _type_name = "HIL_GPS"

    def make_msg(self):
        vel = self.data['velocity']
        gnd_speed = sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2])
        if self.data['heading'] == float('inf'):
            cog = 65535
        else:
            cog = int(self.data['heading']* 100)
        self._msg = mavlink.MAVLink_hil_gps_message(
                self.time_since_boot(),
                3,
                int(self.data['latitude'] * 1e7),
                int(self.data['longitude'] * 1e7),
                int(self.data['altitude'] * 1000),
                100, 100,
                int(gnd_speed * 100),
                int(vel[1] * 100),
                int(vel[0] * 100),
                int(- vel[2] * 100),
                cog,
                10)
