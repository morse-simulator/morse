import logging; logger = logging.getLogger("morse." + __name__)
import random
from collections import defaultdict

from morse.helpers.components import add_property
from morse.modifiers.abstract_modifier import AbstractModifier

class IMUNoiseModifier(AbstractModifier):
    """
    This modifier allows to simulate Gaussian noise for accelerometer and
    gyroscope sensors of an IMU.
    No bias is modeled so far.
    """

    _name = "IMUNoise"
    
    add_property('_gyro_std_dev', {'x': 0.5, 'y': 0.5, 'z': 0.5}, "gyro_std", type = "dict", 
                 doc = "Standard deviation for noise applied to angular velocities as dictionary with x,y,z as floats")
    add_property('_accel_std_dev', {'x': 0.5, 'y': 0.5, 'z': 0.5}, "accel_std", type = "dict", 
                 doc="Standard deviation for noise applied to linear accelerations as dictionary with x,y,z as floats")
    
    def initialize(self):
        gyro_std = self.parameter("gyro_std", default=0.5)
        if isinstance(gyro_std, dict):
            self._gyro_std_dev = defaultdict(lambda: 0.0, gyro_std)
        else:
            self._gyro_std_dev = {'x': float(gyro_std), 'y': float(gyro_std), 'z': float(gyro_std)}
        accel_std = self.parameter("accel_std", default=0.5)
        if isinstance(accel_std, dict):
            self._accel_std_dev = defaultdict(lambda: 0.0, accel_std)
        else:
            self._accel_std_dev = {'x': float(accel_std), 'y': float(accel_std), 'z': float(accel_std)}
        logger.info("IMU Noise standard deviations: gyro x:%.4f, y:%.4f, z:%.4f,  accel x:%.4f, y:%.4f, z:%.4f,"
                    % (self._gyro_std_dev['x'], self._gyro_std_dev['y'], self._gyro_std_dev['z'],
                       self._accel_std_dev['x'], self._accel_std_dev['y'], self._accel_std_dev['z']))

    def modify(self):
        try:
            self.data['angular_velocity'][0] = \
                random.gauss(self.data['angular_velocity'][0], self._gyro_std_dev['x'])
            self.data['linear_acceleration'][0] = \
                random.gauss(self.data['linear_acceleration'][0], self._accel_std_dev['x'])
            self.data['angular_velocity'][1] = \
                random.gauss(self.data['angular_velocity'][1], self._gyro_std_dev['y'])
            self.data['linear_acceleration'][1] = \
                random.gauss(self.data['linear_acceleration'][1], self._accel_std_dev['y'])
            self.data['angular_velocity'][2] = \
                random.gauss(self.data['angular_velocity'][2], self._gyro_std_dev['z'])
            self.data['linear_acceleration'][2] = \
                random.gauss(self.data['linear_acceleration'][2], self._accel_std_dev['z'])
        except KeyError as detail:
            self.key_error(detail)
