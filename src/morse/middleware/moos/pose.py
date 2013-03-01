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
