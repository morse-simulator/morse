import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSNotifier
from morse.core import blenderapi
import json

class multiStaticSimNotifier(MOOSNotifier):
    """ Notify MultiStaticSim """

    def default(self, ci = 'unused'):

        # Don't send any message
        # if there's no lidar pose data
        publish_list = self.data['node_pose']
        
        # printing the list using loop 
        for x in range(len(publish_list)):

            str1 = publish_list[x]
            json_data = json.loads(str1)
            #print(json_data['node_name'])
            #logger.debug('lidarNotifier is publishing!')
            ts = self.data['timestamp']

            # Acomms message
            msg_name = json_data['node_name'].upper() + '_UPDATE'
            self.notify(msg_name, str1 ,ts)

    def update_morse_data(self):
        logger.debug('multiStaticSimNotifier.update_morse_data() called.')
