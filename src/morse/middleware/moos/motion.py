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
        # register for position variables from the database
        self.m.Register("pX")
        self.m.Register("pY")
        self.m.Register("pZ")
        self.m.Register("pRoll")
        self.m.Register("pPitch")
        self.m.Register("pYaw")

    def default(self, ci='unused'):
        current_time = pymoos.MOOSCommClient.MOOSTime()
        # get latest mail from the MOOS comm client
        messages = self.m.FetchRecentMail()

        new_information = False

        for message in messages:
            # look for command messages
            if (message.GetKey() =="cVelocity") and (message.IsDouble()):
                self.data['v'] = message.GetDouble() # command linear velocity [m/s]
                new_information = True
            elif  (message.GetKey()=="cYawRate") and (message.IsDouble()):
                self.data['w'] = message.GetDouble() # command angular velocity [m/s]
                new_information = True
            elif  (message.GetKey() =="cSteer") and (message.IsDouble()):
                self.data['steer'] = message.GetDouble() # command steer angle [deg]
                new_information = True
            elif  (message.GetKey( )=="cThrottle") and (message.IsDouble()):
                self.data['force'] = message.GetDouble() # command engine force
                new_information = True
            elif  (message.GetKey() =="cBrake") and (message.IsDouble()):
                self.data['brake'] = message.GetDouble() # command angular velocity [m/s]
                new_information = True
            # look for position messages
            elif  (message.GetKey() =="pX") and (message.IsDouble()):
                self.data['x'] = message.GetDouble() # robot X position [m]
                new_information = True
            elif  (message.GetKey() =="pY") and (message.IsDouble()):
                self.data['y'] = message.GetDouble() # robot Y position [m]
                new_information = True
            elif  (message.GetKey() =="pZ") and (message.IsDouble()):
                self.data['z'] = message.GetDouble() # robot Z position [m]
                new_information = True
            elif  (message.GetKey() =="pRoll") and (message.IsDouble()):
                self.data['roll'] = message.GetDouble() # robot roll [rad]
                new_information = True
            elif  (message.GetKey() =="pPitch") and (message.IsDouble()):
                self.data['pitch'] = message.GetDouble() # robot pitch [rad]
                new_information = True
            elif  (message.GetKey() =="pYaw") and (message.IsDouble()):
                self.data['yaw'] = message.GetDouble() # robot yaw [rad]
                new_information = True

        return new_information

