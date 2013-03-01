import logging; logger = logging.getLogger("morse." + __name__)
import pymoos.MOOSCommClient
from morse.middleware.moos import AbstractMOOS

class GPSNotifier(AbstractMOOS):
    """ Notify GPS """

    def default(self, ci='unused'):
        cur_time = pymoos.MOOSCommClient.MOOSTime()
        
        self.m.Notify('zEast', self.data['x'], cur_time)
        self.m.Notify('zNorth', self.data['y'], cur_time)
        self.m.Notify('zHeight', self.data['z'], cur_time)
