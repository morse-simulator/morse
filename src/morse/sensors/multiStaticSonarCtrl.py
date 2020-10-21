import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSNotifier
from morse.core import blenderapi
import json

class multiStaticSimNotifier(MOOSNotifier):
    """ Notify MultiStaticSim """

    def default(self, ci = 'unused'):

        # Don't send any message
        # if there's no lidar pose data
        str = self.data['node_pose']

        #logger.debug('lidarNotifier is publishing!')
        ts = self.data['timestamp']

        # Acomms message
        msg_name = self.data['node_name'].upper() + '_UPDATE'
        self.notify(msg_name, str ,ts)

    def update_morse_data(self):
        logger.debug('multiStaticSimNotifier.update_morse_data() called.')
