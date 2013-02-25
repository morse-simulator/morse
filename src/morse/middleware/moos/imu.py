import logging; logger = logging.getLogger("morse." + __name__)
import pymoos.MOOSCommClient
from morse.middleware.moos import AbstractMOOS

class IMUNotifier(AbstractMOOS):
    """ Notify IMU """

    def default(self, ci='unused'):
        curTime=pymoos.MOOSCommClient.MOOSTime()

        vel=self.data['velocity']
        acc=self.data['acceleration']

        # post angular rates
        self.m.Notify('zGyroX',vel[3],curTime)
        self.m.Notify('zGyroY',vel[4],curTime)
        self.m.Notify('zGyroZ',vel[5],curTime)

        # post accelerations
        self.m.Notify('zAccelX',acc[0],curTime)
        self.m.Notify('zAccelY',acc[1],curTime)
        self.m.Notify('zAccelZ',acc[2],curTime)
