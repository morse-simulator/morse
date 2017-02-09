import logging; logger  =  logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSNotifier

class IMUNotifier(MOOSNotifier):
    """ Notify IMU """

    def default(self,  ci = 'unused'):

        vel = self.data['angular_velocity']
        acc = self.data['linear_acceleration']

        # post angular rates
        self.notify('MORSE_IMU_GYRO_X', vel[0])
        self.notify('MORSE_IMU_GYRO_Y', vel[1])
        self.notify('MORSE_IMU_GYRO_Z', vel[2])

        # post accelerations
        self.notify('MORSE_IMU_ACCEL_X', acc[0])
        self.notify('MORSE_IMU_ACCEL_Y', acc[1])
        self.notify('MORSE_IMU_ACCEL_Z', acc[2])
