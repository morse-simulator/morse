import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import *
from platine.struct import *
from math import degrees

class PlatinePoster(PocolibsDataStreamOutput):
    def initialize(self):
        super(self.__class__, self).initialize(PLATINE_STATES)

        self.obj = PLATINE_STATES()

    def default(self, ci):
        self.obj.stateRad.pos.pan = self.data['pan']
        self.obj.stateRad.pos.tilt = self.data['tilt']
        self.obj.stateDeg.pos.pan = degrees(self.data['pan'])
        self.obj.stateDeg.pos.tilt = degrees(self.data['tilt'])
        self.write(self.obj)

