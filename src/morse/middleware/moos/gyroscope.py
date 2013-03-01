import logging; logger = logging.getLogger("morse." + __name__)
import pymoos.MOOSCommClient
from morse.middleware.moos import AbstractMOOS

class GyroscopeNotifier(AbstractMOOS):
    """ Notify Gyroscope """

    def default(self, ci = 'unused'):
        cur_time = pymoos.MOOSCommClient.MOOSTime()
        
        self.m.Notify('zYaw', self.data['yaw'], cur_time)
        self.m.Notify('zRoll', self.data['roll'], cur_time)
        self.m.Notify('zPitch', self.data['pitch'], cur_time)
