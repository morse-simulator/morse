import logging; logger = logging.getLogger("morse." + __name__)
import pymoos.MOOSCommClient
from morse.middleware.moos import AbstractMOOS
from morse.core import blenderapi

class LIDARNotifier(AbstractMOOS):
    """ Notify LIDAR """

    def initialize(self):
        AbstractMOOS.initialize(self)
        # post lidar settings to the database only once at startup
        curTime=blenderapi.persistantstorage().current_time
        self.m.Notify('sScanAngle',self.component_instance.bge_object['scan_window'],curTime) 
        self.m.Notify('sScanResolution',self.component_instance.bge_object['resolution'],curTime) 
        self.m.Notify('sScanRange',self.component_instance.bge_object['laser_range'],curTime)

    def default(self, ci='unused'):
        curTime=pymoos.MOOSCommClient.MOOSTime()
        num_readings = self.component_instance.bge_object['scan_window'] / self.component_instance.bge_object['resolution']

        # build string of the form: '[1x180]{4.9,29.2,...,2.98}'
        # replace [] in python string conversion to {} expected by MOOS parsing code
        laserscan = str(self.data['range_list']).replace('[','{').replace(']','}')
        # add array size to beginning of string
        #laserscan = '[1x' + '{0:.0}'.format(num_readings) + ']'+laserscan
        laserscan = '[1x' + '%.0f' % num_readings + ']'+laserscan

        self.m.Notify('zLidarDist',laserscan,curTime)
