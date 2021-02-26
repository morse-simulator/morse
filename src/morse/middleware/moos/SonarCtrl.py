import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSSubscriber, MOOSNotifier
from morse.core import blenderapi
try:
    from pymoos import pymoos
except:
    import pymoos
class sonarReader(MOOSSubscriber):
    """ Read sonar commands and update local data. """

    def initialize(self):

        # initialize the parent class
        MOOSSubscriber.initialize(self)
        
        # register for control variables from the database
        self.register_message_to_queue('SONAR_STATUS','sonar_status_queue', self.on_sonar_msgs)

    def on_sonar_msgs(self, msg):
        if (msg.key() == 'SONAR_STATUS') and (msg.is_string()):
            self.data['status'] = msg.string()

        self._new_messages = True

        return True

class sonarNotifier(MOOSNotifier):
    """ Notify Sonar """

    def default(self, ci = 'unused'):

        #logger.debug('SonarNotifier is publishing!')
        # ts = pymoos.time()
        ts = self.data['timestamp']

        # Don't send any message
        # if there's no pose data
        str = self.data['pose']

        if str:

            # Construct the MOOS message name
            msg_name = self.data['object'].upper() + '_PING'
            self.notify(msg_name,str,ts)

            # Cancel message for next cycle
            self.data['pose'] = ''

    def update_morse_data(self):
        logger.debug('SonarNotifier.update_morse_data() called.')
