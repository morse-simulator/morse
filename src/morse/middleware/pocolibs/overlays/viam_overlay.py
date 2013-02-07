import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import service, async_service, interruptible
from morse.core.overlay import MorseOverlay
from morse.core import status
from morse.middleware.pocolibs_datastream import DummyPoster

class ViamModule(MorseOverlay):
    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(overlaid_object)
        self._cntrl = DummyPoster("viamCntrl")

    def Acquire_cb(self, answer):
        status, res = answer
        return (status, [ self._bench, self._n])

    @service
    def Init(self, *args):
        pass

    @service
    def Configure(self, *args):
        pass

    @service
    def Reset(self, *args):
        pass

    @service
    def DriverLoad(self, *args):
        pass

    @service
    def BusPrint(self, *args):
        pass

    @service 
    def BankCreate(self, *args):
        pass

    @service
    def CameraCreate(self, *args):
        pass

    @service 
    def BankAddCamera(self, *args):
        pass

    @service
    def PushFormatFilter(self, *args):
        pass

    @service
    def UpdateFormatFilter(self, *args):
        pass

    @service
    def PushGeoFilter(self, *args):
        pass

    @service
    def UpdateGeoFilter(self, *args):
        pass

    @service
    def PushLumFilter(self, *args):
        pass

    @service
    def UpdateLumFilter(self, *args):
        pass

    @service
    def PushColorFilter(self, *args):
        pass

    @service
    def UpdateColorFilter(self, *args):
        pass

    @service
    def PushImProcFilter(self, *args):
        pass

    @service
    def UpdateImProcFilter(self, *args):
        pass

    @service
    def PushMiscFilter(self, *args):
        pass

    @service
    def UpdateMiscFilter(self, *args):
        pass

    @service 
    def FilterList(self, *args):
        pass

    @service
    def FilterGetCap(self, *args):
        pass

    @service
    def CameraListHWModes(self, *args):
        pass

    @service
    def CameraSetHWMode(self, *args):
        pass

    @service
    def Display(self, *args):
        pass

    @service
    def Calibrate(self, *args):
        pass

    @service
    def CalibrationIO(self, *args):
        pass

    @service
    def Save(self, *args):
        pass


    @interruptible
    @async_service
    def Acquire(self, bench, n):
        n = int(n)
        self._n = n
        self._bench = bench
        if n == 0:
            n = -1
        self.overlaid_object.capture(self.chain_callback(self.Acquire_cb), n)

    @async_service
    def Stop(self, bench):
        self.completed(status.SUCCESS)

    def name(self):
        return "viam"

