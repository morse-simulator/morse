import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import service, async_service, interruptible
from morse.core.overlay import MorseOverlay
from morse.core import status
from morse.middleware.pocolibs_datastream import DummyPoster

class StereopixelModule(MorseOverlay):
    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        MorseOverlay.__init__(self, overlaid_object)
        self._cntrl = DummyPoster("stereopixelCntrl")

    @service
    def Init(self, *args):
        pass

    @service
    def GetCorrelationParams(self, *args):
        pass

    @service
    def SetCorrelationParams(self, *args):
        pass

    @service
    def GetBlobFilterParams(self, *args):
        pass

    @service
    def SetBlobFilterParams(self, *args):
        pass

    @service 
    def GetFrameCoord(self, *args):
        pass

    @service
    def SetFrameCoord(self, *args):
        pass

    @service 
    def SetSaveParams(self, *args):
        pass

    @service
    def GetSaveParams(self, *args):
        pass

    @service
    def SaveDisparity(self, *args):
        pass

    @service
    def CVDumpDisparity(self, *args):
        pass

    @service
    def Sav3DImage(self, *args):
        pass

    @interruptible
    @async_service
    def Compute(self):
        self.overlaid_object.capture(self.chain_callback(), 1)

    def name(self):
        return "stereopixel"

