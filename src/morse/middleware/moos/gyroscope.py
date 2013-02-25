import logging; logger = logging.getLogger("morse." + __name__)
import pymoos.MOOSCommClient
from morse.middleware.moos import AbstractMOOS

class GyroscopeNotifier(AbstractMOOS):
    """ Notify Gyroscope """

    def default(self, ci='unused'):
        curTime=pymoos.MOOSCommClient.MOOSTime()
        
        self.m.Notify('zYaw',self.data['yaw'],curTime)
        self.m.Notify('zRoll',self.data['roll'],curTime)
        self.m.Notify('zPitch',self.data['pitch'],curTime)
