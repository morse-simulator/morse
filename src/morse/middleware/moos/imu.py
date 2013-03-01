import logging; logger  =  logging.getLogger("morse." + __name__)
import pymoos.MOOSCommClient
from morse.middleware.moos import AbstractMOOS

class IMUNotifier(AbstractMOOS):
    """ Notify IMU """

    def default(self,  ci = 'unused'):
        cur_time = pymoos.MOOSCommClient.MOOSTime()

        vel = self.data['velocity']
        acc = self.data['acceleration']

        # post angular rates
        self.m.Notify('zGyroX', vel[3], cur_time)
        self.m.Notify('zGyroY', vel[4], cur_time)
        self.m.Notify('zGyroZ', vel[5], cur_time)

        # post accelerations
        self.m.Notify('zAccelX', acc[0], cur_time)
        self.m.Notify('zAccelY', acc[1], cur_time)
        self.m.Notify('zAccelZ', acc[2], cur_time)
