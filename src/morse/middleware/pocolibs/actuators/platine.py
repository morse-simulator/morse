from morse.middleware.pocolibs_datastream import PocolibsDataStreamInput, poster_name
from pom.struct import *
from platine.struct import *

class PlatinePoster(PocolibsDataStreamInput):
    def initialize(self):
        PocolibsDataStreamInput.initialize(self, POM_SE_POSTER)

    def default(self, component):
        pos = self.read()
        if pos:
            euler = pos.seConfig.euler
            self.data['pan'] = euler.yaw
            self.data['tilt'] = euler.pitch

class PlatineAxisPoster(PocolibsDataStreamInput):
    def initialize(self):
        PocolibsDataStreamInput.initialize(self, PLATINE_AXIS_STR)

    def default(self, component):
        pos = self.read()
        if pos:
            self.data['pan'] = pos.pan
            self.data['tilt'] = pos.tilt

