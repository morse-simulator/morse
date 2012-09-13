import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import service, async_service, interruptible
from morse.core.overlay import MorseOverlay
from morse.core import status
from morse.middleware.pocolibs.actuators.genpos import GenPosPoster, PosterNotFound
from morse.middleware.pocolibs.sensors.General_Poster.ors_poster import new_poster

class RflexModule(MorseOverlay):
    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(overlaid_object)
        self._clean_track = False
        new_poster("rflexCntrl", 4)

    def interrupt(self):
        self.overlaid_object.stop()
        if self._clean_track:
            self.overlaid_object.input_functions.pop()
        super(RflexModule, self).interrupt()

    @service
    def InitClient(self, *args):
        pass

    @service
    def EndClient(self, *args):
        pass

    @service
    def SetWdogRef(self, *args):
        pass

    @service
    def GetWdogRef(self, *args):
        pass

    @service
    def SetMode(self, mode):
        self._mode = mode

    @service
    def GetMode(self):
        return (status.SUCCESS, self._mode)

    @service
    def PomTagging(self, *args):
        pass

    @service 
    def SetPos(self, *args):
        pass

    @service 
    def SetPosFromMEPoster(self, *args):
        pass

    @service 
    def SetVarparams(self, *args):
        pass

    @async_service
    def Stop(self, *args):
        self.overlaid_object.stop()
        self.completed(status.SUCCESS)

    @async_service
    def TrackEnd(self, *args):
        self.overlaid_object.stop()
        self.completed(status.SUCCESS)

    @async_service
    def GotoSpeed(self, numRef, updatedPeriod, v, vt, w, *args):
        self.overlaid_object.set_speed(float(v), float(w))
        self.completed(status.SUCCESS)

    @interruptible
    @async_service
    def TrackSpeedStart(self, poster_name):
        try:
            poster = GenPosPoster(poster_name)
        except PosterNotFound:
            return self.completed(status.FAILED, ["POSTER_NOT_FOUND"])

        self._clean_track = True
        self.overlaid_object.input_functions.append(poster.read)


    @service
    def GetGeoConfig(self, *args):
        pass

    @service
    def SetGeoConfig(self, *args):
        pass

    @service
    def SonarOn(self, *args):
        pass

    @service
    def SonarOff(self, *args):
        pass

    @service
    def BrakeOn(self, *args):
        pass

    @service
    def BrakeOff(self, *args):
        pass

    @service
    def GetJoystick(self, *args):
        pass

    @service
    def MonitorBattery(self, *args):
        pass

    @service
    def Gyro(self, *args):
        pass

    @service
    def GetGyro(self, *args):
        pass

    @service 
    def SetOdometryMethod(self, *args):
        pass

    @service
    def Log(self, *args):
        pass

    @service
    def StopLog(self, *args):
        pass

    def name(self):
        return "rflex"

