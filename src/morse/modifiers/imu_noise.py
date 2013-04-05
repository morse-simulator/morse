import logging; logger = logging.getLogger("morse." + __name__)
import random

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
            self._gyro_std_dev = gyro_std
        else:
            self._gyro_std_dev = {'x': float(gyro_std), 'y': float(gyro_std), 'z': float(gyro_std)}
        accel_std = self.parameter("accel_std", default=0.5)
        if isinstance(accel_std, dict):
            self._accel_std_dev = accel_std
        else:
            self._accel_std_dev = {'x': float(accel_std), 'y': float(accel_std), 'z': float(accel_std)}
        logger.info("IMU Noise standard deviations: gyro x:%.4f, y:%.4f, z:%.4f,  accel x:%.4f, y:%.4f, z:%.4f,",
                    self._gyro_std_dev.get('x', 0), self._gyro_std_dev.get('y', 0), self._gyro_std_dev.get('z'),
                    self._accel_std_dev.get('x', 0), self._accel_std_dev.get('y', 0), self._accel_std_dev.get('z', 0))

    def modify(self):
        axes = ['x', 'y', 'z']
        for i in range(0, 3):
            if axes[i] in self._gyro_std_dev:
                self.data['angular_velocity'][i] = \
                    random.gauss(self.data['angular_velocity'][i], self._gyro_std_dev[axes[i]])
            if axes[i] in self._accel_std_dev:
                self.data['linear_acceleration'][i] = \
                    random.gauss(self.data['linear_acceleration'][i], self._accel_std_dev[axes[i]])

