import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSSubscriber, MOOSNotifier
from morse.core import blenderapi
import json

class lidarReader(MOOSSubscriber):
    """ Read lidar commands and update local data. """

    def initialize(self):

        # initialize the parent class
        MOOSSubscriber.initialize(self)
        
        # register for control variables from the database
        self.register_message_to_queue('LIDAR_STATUS','lidar_status_queue', self.on_lidar_msgs)

    def on_lidar_msgs(self, msg):
        if (msg.key() == 'LIDAR_STATUS') and (msg.is_string()):
            self.data['status'] = msg.string()

        self._new_messages = True

        return True

class lidarNotifier(MOOSNotifier):
    """ Notify Lidar """

    def default(self, ci = 'unused'):

        # Don't send any message
        # if there's no lidar pose data
        str = self.data['lidar_pose']

        if str:

            #logger.debug('lidarNotifier is publishing!')
            ts = self.data['timestamp']

            # Lidar message
            msg_name = self.data['lidar_name'] + '_TRIGGER'
            self.notify(msg_name, str ,ts)

            # Lidar view message
            self.notify('LIDAR_VIEW', self.data['lidar_view'] ,ts)

            # Cancel message for next cycle
            self.data['lidar_pose'] = ''

    def update_morse_data(self):
        logger.debug('lidarNotifier.update_morse_data() called.')
