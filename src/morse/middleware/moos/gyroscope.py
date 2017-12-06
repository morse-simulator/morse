import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSNotifier

class GyroscopeNotifier(MOOSNotifier):
    """ Notify Gyroscope """

    def default(self, ci = 'unused'):
        logger.debug('GyroscopeNotifier is publising!')
        self.notify('MORSE_GYRO_YAW', self.data['yaw'])
        self.notify('MORSE_GYRO_ROLL', self.data['roll'])
        self.notify('MORSE_GYRO_PITCH', self.data['pitch'])
