import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import service, async_service
from morse.core import status, blenderapi
from morse.core.abstractobject import AbstractObject
from morse.core.morse_time import time_isafter

class TimeServices(AbstractObject):
    def __init__(self):
        super(TimeServices, self).__init__()
        self.time = blenderapi.persistantstorage().time
        self._alarm_time = None

    def name(self):
        return "time"

    @service
    def mode(self):
        """ Return the current time management mode """
        return self.time.name()

    @service
    def now(self):
        """ Return the simulator time, in seconds, since Epoch """
        return self.time.time

    @service
    def statistics(self):
        """
        Return various statistics associated to the specific time
        management mode
        """
        return self.time.statistics()

    @async_service
    def sleep(self, time):
        """
        Sleep  for time seconds

        :param: a float representing the time to wait (in second)
        """

        self._alarm_time = self.time.time + float(time)
        logger.debug('alarm registered for %f' % self._alarm_time)

    def action(self):
        if self._alarm_time and time_isafter(self.time.time, self._alarm_time):
            logger.debug('alarm fired at %f difference %f' % (self.time.time, self._alarm_time - self.time.time))
            self._alarm_time = None
            self.completed(status.SUCCESS)
