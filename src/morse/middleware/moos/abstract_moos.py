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

        if 'moos_host' in self.kwargs:
            self.moos_host = self.kwargs['moos_host']
        else:
            self.moos_host = "127.0.0.1"

        if 'moos_port' in self.kwargs:
            self.moos_port = self.kwargs['moos_port']
        else:
            self.moos_port = 9000

        if 'moos_freq' in self.kwargs:
            self.moos_freq = self.kwargs['moos_freq']
        else:
            self.moos_freq = 10 # [Hz]

        mh = self.moos_host
        mp = self.moos_port
        mf = self.moos_freq

        if not (mh, mp) in AbstractMOOS._moosapps:
            AbstractMOOS._save_messages[mh, mp] = []
            AbstractMOOS._moosapps[mh, mp] = pymoos.MOOSCommClient.MOOSApp()
            AbstractMOOS._moosapps[mh, mp].Run(mh, mp, "uMorse", mf)
            logger.info("\tdatastream: host=%s:port=%d (freq: %.2fHz)"%(mh, mp, mf))
            logger.info("\tnew interface initialized")

        # all instance share the same static MOOSApp according to host and port
        self.m = AbstractMOOS._moosapps[mh, mp]

    def getRecentMail(self):
        """ Get recent messages from MOOS. """
        mh = self.moos_host
        mp = self.moos_port
        messages = self.m.FetchRecentMail()

        # a call to FetchRecentMail empties the mail list in MOOSCommClient.MOOSApp
        # because multiple actuators can share the same MOOSApp instance, 
        # it is necessary to save new mails statically

        # when there are new messages, the static list is updated
        if len(messages) != 0:
            AbstractMOOS._save_messages[mh, mp] = messages
        
        return AbstractMOOS._save_messages[mh, mp]

    def finalize(self):
        """ Kill the morse MOOS app."""
        mh = self.moos_host
        mp = self.moos_port
        if (mh, mp) in AbstractMOOS._moosapps:
            AbstractMOOS._moosapps[mh, mp].Close()
            AbstractMOOS._moosapps.pop((mh, mp))
            AbstractMOOS._save_messages.pop((mh, mp))
            logger.info("MOOS datastream finalized: %s:%d"%(mh, mp))


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