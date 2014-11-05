import logging; logger = logging.getLogger("morse." + __name__)
import pymoos.MOOSCommClient
from morse.middleware.moos import AbstractMOOS

class LightReader(AbstractMOOS):
    """ Read light commands. """

    def initialize(self):
        AbstractMOOS.initialize(self)
        # register for control variables from the database
        self.m.Register("cLight")

    def default(self, ci='unused'):
        current_time = pymoos.MOOSCommClient.MOOSTime()
        # get latest mail from the MOOS comm client
        messages = self.m.FetchRecentMail()

        new_information = False

        for message in messages:
            # look for command messages
            if (message.GetKey() == "cLight"):
                self.data['emit'] = (message.GetString()=="true")
                new_information = True

        return new_information