import logging; logger = logging.getLogger("morse." + __name__)
import pymoos.MOOSCommClient
from morse.middleware.moos import AbstractMOOS
from morse.core import blenderapi

class LIDARNotifier(AbstractMOOS):
    """ Notify LIDAR """

    def initialize(self):
        AbstractMOOS.initialize(self)
        # post lidar settings to the database only once at startup
        cur_time=blenderapi.persistantstorage().current_time
        self.m.Notify('sScanAngle',
            self.component_instance.scan_window, cur_time)
        self.m.Notify('sScanResolution',
            self.component_instance.resolution, cur_time)
        self.m.Notify('sScanRange',
            self.component_instance.laser_range, cur_time)

    def default(self,  ci='unused'):
        cur_time=pymoos.MOOSCommClient.MOOSTime()
        num_readings = self.component_instance.scan_window / \
                      self.component_instance.resolution

        # build string of the form: '[1x180]{4.9, 29.2, ..., 2.98}'
        # replace [] in python string conversion to {} expected by MOOS parsing code
        laserscan = str(self.data['range_list']).replace('[', '{').replace(']', '}')
        # add array size to beginning of string
        #laserscan = '[1x' + '{0:.0}'.format(num_readings) + ']'+laserscan
        laserscan = '[1x' + '%.0f' % num_readings + ']'+laserscan

        self.m.Notify('zLidarDist', laserscan, cur_time)
