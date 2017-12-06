import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSNotifier
from morse.core import blenderapi

class LIDARNotifier(MOOSNotifier):
    """ Notify LIDAR """

    def initialize(self):
        MOOSNotifier.initialize(self)
        # post lidar settings to the database only once at startup
        self.notify('MORSE_LIDAR_SCANANGLE',
            self.component_instance.scan_window)
        self.notify('MORSE_LIDAR_SCANRESOLUTION',
            self.component_instance.resolution)
        self.notify('MORSE_LIDAR_SCANRANGE',
            self.component_instance.laser_range)

    def default(self,  ci='unused'):
        num_readings = self.component_instance.scan_window / \
                      self.component_instance.resolution

        # build string of the form: '[1x180]{4.9, 29.2, ..., 2.98}'

        # replace [] in python string conversion to {} expected by MOOS parsing code
        laserscan = str(self.data['range_list']).replace('[', '{').replace(']', '}')
        # add array size to beginning of string
        #laserscan = '[1x' + '{0:.0}'.format(num_readings) + ']'+laserscan
        laserscan = '[1x' + '%.0f' % num_readings + ']'+laserscan

        self.notify('MORSE_LIDAR_DIST', laserscan)
