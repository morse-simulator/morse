import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import service, async_service, interruptible
from morse.core.overlay import MorseOverlay
from morse.core import status
from morse.middleware.pocolibs_datastream import DummyPoster

class VelodyneModule(MorseOverlay):
    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        MorseOverlay.__init__(self, overlaid_object)
        self._cntrl = DummyPoster("velodyneCntrl")

    @service
    def Init(self, *args):
        pass

    @service
    def GetInitParams(self, *args):
        pass

    @service
    def FOVRemoveHeadingSector(self, *args):
        pass

    @service
    def FOVRemoveLine(self, *args):
        pass

    @service
    def StartAcquisition(self, *args):
        pass

    @service
    def StopAcquisition(self, *args):
        pass

    @service
    def OneShot(self, *args):
        pass

    @service
    def SetTargetDetectionParams(self, *args):
        pass

    @service
    def GetTargetDetectionParams(self, *args):
        pass

    @service
    def SetTargetParams(self, *args):
        pass

    @service
    def GetTargetParams(self, *args):
        pass

    @service
    def StopTargetDetection(self, *args):
        pass

    @service
    def DumpTarget(self, *args):
        pass

    @service
    def SetSaveParams(self, *args):
        pass

    @service
    def SetSaveIm3DParams(self, *args):
        pass

    @service
    def GetSaveParams(self, *args):
        pass

    @service
    def GetSaveIm3DParams(self, *args):
        pass

    @service
    def Save(self, *args):
        pass

    @service
    def SaveIm3d(self, *args):
        pass

    def name(self):
        return "velodyne"

