import logging; logger = logging.getLogger("morse.moos")
import pymoos.MOOSCommClient

from morse.middleware import AbstractDatastream
from morse.core import blenderapi


class AbstractMOOS(AbstractDatastream):
    """ Base class for all MOOS Publishers and Subscribers """
    # used to generate documentation, TODO fill in subclasses
    _type_name = "db entries"
    _type_url = ""
    _moosapps = {}
    _save_messages = {}

    def initialize(self):
        """ Initialize the MOOS app. """
        logger.info("MOOS datastream initialize %s"%self)

        self.moos_host = self.kwargs.get('moos_host', '127.0.0.1')
        self.moos_port = self.kwargs.get('moos_port', 9000)
        self.moos_freq = self.kwargs.get('moos_freq', 10.0)

        key = (self.moos_host, self.moos_port)

        if not key in AbstractMOOS._moosapps:
            AbstractMOOS._save_messages[key] = []
            AbstractMOOS._moosapps[key] = pymoos.MOOSCommClient.MOOSApp()
            AbstractMOOS._moosapps[key].Run(self.moos_host,
                                            self.moos_port,
                                            "uMorse", 
                                            self.moos_freq)
            logger.info("\tdatastream: host=%s:port=%d (freq: %.2fHz)"
                %(self.moos_host, self.moos_port, self.moos_freq))
            logger.info("\tnew interface initialized")

        # all instance share the same static MOOSApp according to host and port
        self.m = AbstractMOOS._moosapps[key]

    def getRecentMail(self):
        """ Get recent messages from MOOS. """
        key = (self.moos_host, self.moos_port)
        messages = self.m.FetchRecentMail()

        # a call to FetchRecentMail empties the mail list in MOOSCommClient.MOOSApp
        # because multiple actuators can share the same MOOSApp instance, 
        # it is necessary to save new mails statically

        # when there are new messages, the static list is updated
        if len(messages) != 0:
            AbstractMOOS._save_messages[key] = messages
        
        return AbstractMOOS._save_messages[key]

    def finalize(self):
        """ Kill the morse MOOS app."""
        key = (self.moos_host, self.moos_port)
        if key in AbstractMOOS._moosapps:
            AbstractMOOS._moosapps[key].Close()
            AbstractMOOS._moosapps.pop(key)
            AbstractMOOS._save_messages.pop(key)
            logger.info("MOOS datastream finalized: %s:%d"
                %(self.moos_host, self.moos_port))


class StringPublisher(AbstractMOOS):
    """ Publish a string containing a printable representation of the
    local data. """

    def default(self, ci='unused'):
        logger.debug("Posting message to the MOOS database.")
        current_time = blenderapi.persistantstorage().current_time
        #iterate through all objects of the component_instance and post the data
        for variable, data in self.data.items():
            name = "%s_%s" % (self.component_name, variable)
            logger.debug("name: %s, type: %s, data: %s"%
                                (name, type(data), str(data)))
            self.m.Notify(name, str(data), current_time)


class StringReader(AbstractMOOS):
    """ Log messages. """

    def default(self, ci='unused'):
        # get latest mail from the MOOS comm client
        messages = self.m.FetchRecentMail()

        # log messages
        for message in messages:
            logger.info("message: %s" % str(message))