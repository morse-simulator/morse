import logging; logger = logging.getLogger("morse." + __name__)
#logger.setLevel(logging.DEBUG)
from morse.core.services import service, async_service, interruptible
from morse.core.overlay import MorseOverlay
from morse.core import status
from functools import partial

from morse.middleware.pocolibs.actuators.platine import PlatinePoster, PosterNotFound
from morse.middleware.pocolibs.sensors.General_Poster.ors_poster import new_poster

import math

class PlatineModule(MorseOverlay):
    def __init__(self, overlaid_object):
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(overlaid_object)
        self._clean_track = False
        self._rot = 1.0
        new_poster("platineCntrl", 4)

    def _dummy_completion(self, *args):
        logger.debug("enter in _dummy_completion")
        pass

    def _compute_real_angle(self, deg, absolute, pan, tilt):
        if (deg == 'PLATINE_DEG' or deg == '1'):
            logger.debug("converting degree to radian")
            tilt = math.radians(tilt)
            pan  = math.radians(pan)

        if (absolute == 'PLATINE_RELATIVE' or absolute == '0'):
            logger.debug("calculing absolute coordinate")
            current_pan, current_tilt = self.overlaid_object.get_pan_tilt()
            tilt =  current_tilt + tilt
            pan = current_pan + pan

        return (self._rot * pan), tilt

    def _set_pan_tilt(self, pan, tilt, wait):
        if (wait == 'PLATINE_TRUE' or wait == '1'):
            self.overlaid_object.set_pan_tilt(self.chain_callback(), pan, tilt)
        else:
            self.overlaid_object.set_pan_tilt(partial(self._dummy_completion), pan, tilt)
            self.completed((status.SUCCESS))

    def _interrupt(self):
        pan, tilt = self.overlaid_object.get_pan_tilt()
        self.overlaid_object.local_data['tilt'] = tilt
        self.overlaid_object.local_data['pan'] = pan

    def interrupt(self):
        if self._clean_track:
            self.overlaid_object.input_functions.pop()
            self.completed(status.PREEMPTED)
        self._interrupt()
        super(PlatineModule, self).interrupt()

    @service
    def InitDriver(self, serial, baud, rot):
        self._serial = serial
        self._baud = baud
        self._rot = float(rot)

    @service
    def GetSerialParams(self):
        return self._serial, self._baud, self._rot
    
    @interruptible
    @async_service
    def CmdPosCoord(self, deg, absolute, pan, tilt, wait, dummy):
        r_pan, r_tilt =  self._compute_real_angle(deg, absolute, float(pan), - float(tilt))
        self._set_pan_tilt(r_pan, r_tilt, wait)

    @interruptible
    @async_service
    def CmdPosTilt(self, deg, absolute, tilt, wait, dummy):
        c_pan, c_tilt = self.overlaid_object.get_pan_tilt()
        r_pan, r_tilt =  self._compute_real_angle(deg, absolute, \
                         0.0, - float(tilt))
        self._set_pan_tilt(c_pan, r_tilt, wait)

    @interruptible
    @async_service
    def CmdPosPan(self, deg, absolute, pan, wait, dummy):
        c_pan, c_tilt = self.overlaid_object.get_pan_tilt()
        r_pan, r_tilt = self._compute_real_angle(deg, absolute, float(pan), 0.0)
        self._set_pan_tilt(r_pan, c_tilt, wait)

    @interruptible
    @async_service
    def AimAtTargetPoint(self, p1, p2, p3, p4, p5, p6, frame, p7, x, y, z):
        self.overlaid_object.look_at_point(self.chain_callback(), float(x), float(y), float(z))

    @interruptible
    @async_service
    def TrackPos(self, poster_name):
        try:
            poster = PlatinePoster(poster_name)
        except PosterNotFound:
            return self.completed(status.FAILED, ["POSTER_NOT_FOUND"])

        self._clean_track = True
        self.overlaid_object.input_functions.append(poster.read)

    @interruptible
    @async_service
    def Stop(self):
        self._interrupt()
        self.completed((status.SUCCESS))

    def name(self):
        return "platine"
