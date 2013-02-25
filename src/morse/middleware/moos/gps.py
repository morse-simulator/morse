import logging; logger = logging.getLogger("morse." + __name__)
import pymoos.MOOSCommClient
from morse.middleware.moos import AbstractMOOS

class GPSNotifier(AbstractMOOS):
    """ Notify GPS """

    def default(self, ci='unused'):
        curTime=pymoos.MOOSCommClient.MOOSTime()
        
        self.m.Notify('zEast',self.data['x'],curTime)
        self.m.Notify('zNorth',self.data['y'],curTime)
        self.m.Notify('zHeight',self.data['z'],curTime)
