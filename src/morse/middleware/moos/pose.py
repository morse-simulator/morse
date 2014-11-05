import logging; logger = logging.getLogger("morse." + __name__)
import pymoos.MOOSCommClient
from morse.middleware.moos import AbstractMOOS
from morse.core import blenderapi

class PoseNotifier(AbstractMOOS):
    """ Notify Pose """

    def default(self,  ci='unused'):
        cur_time=pymoos.MOOSCommClient.MOOSTime()

        # post the simulation time so that it can be synced to MOOSTime
        self.m.Notify('actual_time', 
                      blenderapi.persistantstorage().current_time, cur_time)
        # post the robot position
        self.m.Notify('simEast', self.data['x'], cur_time)
        self.m.Notify('simNorth', self.data['y'], cur_time)
        self.m.Notify('simHeight', self.data['z'], cur_time)
        self.m.Notify('simYaw', self.data['yaw'], cur_time)
        self.m.Notify('simRoll', self.data['roll'], cur_time)
        self.m.Notify('simPitch', self.data['pitch'], cur_time)

class PoseReader(AbstractMOOS):
    """ Read pose commands and update local data. """

    def initialize(self):
        AbstractMOOS.initialize(self)
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
            # look for position messages
            if  (message.GetKey() == "pX") and (message.IsDouble()):
                self.data['x'] = message.GetDouble() # robot X position [m]
                new_information = True
            elif  (message.GetKey() == "pY") and (message.IsDouble()):
                self.data['y'] = message.GetDouble() # robot Y position [m]
                new_information = True
            elif  (message.GetKey() == "pZ") and (message.IsDouble()):
                self.data['z'] = message.GetDouble() # robot Z position [m]
                new_information = True
            elif  (message.GetKey() == "pRoll") and (message.IsDouble()):
                self.data['roll'] = message.GetDouble() # robot roll [rad]
                new_information = True
            elif  (message.GetKey() == "pPitch") and (message.IsDouble()):
                self.data['pitch'] = message.GetDouble() # robot pitch [rad]
                new_information = True
            elif  (message.GetKey() == "pYaw") and (message.IsDouble()):
                self.data['yaw'] = message.GetDouble() # robot yaw [rad]
                new_information = True

        return new_information