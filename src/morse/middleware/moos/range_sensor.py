
import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.moos import MOOSNotifier

class RangeSensorNotifier(MOOSNotifier):
    """ Notify RangeSensor """

    def default(self, ci = 'unused'):
        logger.debug('RangeSensorNotifier is publising!')
        self.notify(self.data["name"]+'_RANGE', self.data['range'])
