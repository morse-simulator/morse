import logging; logger = logging.getLogger("morse." + __name__)
import pymoos.MOOSCommClient
from morse.middleware.moos import AbstractMOOS

class MotionReader(AbstractMOOS):
    """ Read motion commands and update local data. """

    def initialize(self):
        AbstractMOOS.initialize(self)
        # register for control variables from the database
        self.m.Register("cVelocity")
        self.m.Register("cYawRate")
        self.m.Register("cSteer")
        self.m.Register("cThrottle")
        self.m.Register("cBrake")

    def default(self, ci='unused'):
        current_time = pymoos.MOOSCommClient.MOOSTime()
        # get latest mail from the MOOS comm client
        messages = self.m.FetchRecentMail()

        new_information = False

        for message in messages:
            # look for command messages
            if (message.GetKey() == "cVelocity") and (message.IsDouble()):
                self.data['v'] = message.GetDouble() # command linear velocity [m/s]
                new_information = True
            elif  (message.GetKey() == "cYawRate") and (message.IsDouble()):
                self.data['w'] = message.GetDouble() # command angular velocity [m/s]
                new_information = True
            elif  (message.GetKey() == "cSteer") and (message.IsDouble()):
                self.data['steer'] = message.GetDouble() # command steer angle [deg]
                new_information = True
            elif  (message.GetKey() == "cThrottle") and (message.IsDouble()):
                self.data['force'] = message.GetDouble() # command engine force
                new_information = True
            elif  (message.GetKey() == "cBrake") and (message.IsDouble()):
                self.data['brake'] = message.GetDouble() # command angular velocity [m/s]
                new_information = True

        return new_information

