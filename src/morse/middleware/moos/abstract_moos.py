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
        if not AbstractMOOS._moosapp:
            m = pymoos.MOOSCommClient.MOOSApp()
            #m.SetOnConnectCallBack( m.DoRegistrations )
            #m.SetOnMailCallBack( m.MailCallback )

            logger.info("%s" % m.GetLocalIPAddress())

            fundamental_frequency = 10 # [Hz]
            m.Run( "127.0.0.1", 9000, "MORSE_SIM", fundamental_frequency)
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

