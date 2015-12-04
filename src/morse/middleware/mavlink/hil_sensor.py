import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.mavlink.abstract_mavlink import MavlinkSensor
from pymavlink.dialects.v10 import common as mavlink
from math import sqrt

class HILSensor(MavlinkSensor):
    _type_name = "HIL_SENSOR"

    def make_msg(self):
        imu = self.data[self.component_name + '.imu']
        airspeed = self.data[self.component_name + '.airspeed']
        barometer = self.data[self.component_name + '.barometer']
        thermomether = self.data[self.component_name + '.thermomether']

        self._msg = mavlink.MAVLink_hil_sensor_message(
                self.time_since_boot(),
                imu['linear_acceleration'][0],
                imu['linear_acceleration'][1],
                imu['linear_acceleration'][2],
                imu['angular_velocity'][0],
                imu['angular_velocity'][1],
                imu['angular_velocity'][2],
                imu['magnetic_field'][0],
                imu['magnetic_field'][1],
                imu['magnetic_field'][2],
                barometer['pressure'] * 0.00001,
                airspeed['diff_pressure'] * 0.00001,
                self.component_instance.position_3d.z,
                thermomether['temperature'],
                4095)
