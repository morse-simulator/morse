import logging; logger = logging.getLogger("morse." + __name__)
import pymoos.MOOSCommClient
from morse.middleware.moos import AbstractMOOS
from morse.core import blenderapi

class PoseNotifier(AbstractMOOS):
    """ Notify Pose """

    def default(self, ci='unused'):
        curTime=pymoos.MOOSCommClient.MOOSTime()

        # post the simulation time so that it can be synced to MOOSTime
        self.m.Notify('actual_time',blenderapi.persistantstorage().current_time,curTime)
        # post the robot position
        self.m.Notify('simEast',self.data['x'],curTime)
        self.m.Notify('simNorth',self.data['y'],curTime)
        self.m.Notify('simHeight',self.data['z'],curTime)
        self.m.Notify('simYaw',self.data['yaw'],curTime)
        self.m.Notify('simRoll',self.data['roll'],curTime)
        self.m.Notify('simPitch',self.data['pitch'],curTime)
