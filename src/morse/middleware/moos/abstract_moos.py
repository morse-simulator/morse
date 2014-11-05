import logging; logger = logging.getLogger("morse.moos")
import pymoos.MOOSCommClient

from morse.middleware import AbstractDatastream
from morse.core import blenderapi


class AbstractMOOS(AbstractDatastream):
    """ Base class for all MOOS Publishers and Subscribers """
    # used to generate documentation, TODO fill in subclasses
    _type_name = "db entries"
    _type_url = ""
    _moosapp = None

    def initialize(self):
        """ Initialize the MOOS app"""
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

        if not AbstractMOOS._moosapp:
            m = pymoos.MOOSCommClient.MOOSApp()
            #m.SetOnConnectCallBack( m.DoRegistrations )
            #m.SetOnMailCallBack( m.MailCallback )

            logger.info("%s" % m.GetLocalIPAddress())

            m.Run(self.moos_host, self.moos_port, "uMorse", self.moos_freq)
            logger.info("MOOS datastream: host=%s:port=%d"%
                        (self.moos_host, self.moos_port))
            AbstractMOOS._moosapp = m
            logger.info("MOOS datastream interface initialized")
        # all instance share the same static MOOSApp
        self.m = AbstractMOOS._moosapp

    def finalize(self):
        """ Kill the morse MOOS app."""
        if AbstractMOOS._moosapp:
            AbstractMOOS._moosapp.Close()
            AbstractMOOS._moosapp = None
        logger.info("MOOS datastream finalize %s"%self)


#
# Example (String)
#

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

