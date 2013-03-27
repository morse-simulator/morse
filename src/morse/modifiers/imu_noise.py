import logging; logger = logging.getLogger("morse." + __name__)
import random

from morse.modifiers.abstract_modifier import AbstractModifier

class IMUNoiseModifier(AbstractModifier):
    def initialize(self):
        self._gyro_std_dev = float(self.parameter("gyro_std", default=0.5))
        self._accel_std_dev = float(self.parameter("accel_std", default=0.5))
        logger.info("IMU Noise standard deviations: gyro %.4f, accel %.4f"
                    % (self._gyro_std_dev, self._accel_std_dev))

    def modify(self):
        try:
            for i in range(0, 3):
                self.data['angular_velocity'][i] = \
                random.gauss(self.data['angular_velocity'][i], self._gyro_std_dev)
                self.data['linear_acceleration'][i] = \
                random.gauss(self.data['linear_acceleration'][i], self._accel_std_dev)
        except KeyError as detail:
            self.key_error(detail)
