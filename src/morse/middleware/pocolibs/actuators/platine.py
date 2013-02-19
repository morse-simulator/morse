from morse.middleware.pocolibs_datastream import PocolibsDataStreamInput, poster_name
from pom.struct import *
from platine.struct import *

class PlatinePoster(PocolibsDataStreamInput):
    _type_name = "POM_SE_POSTER"
    _type_url = "http://trac.laas.fr/git/pom-genom/tree/pomStruct.h#n167"

    def initialize(self):
        PocolibsDataStreamInput.initialize(self, POM_SE_POSTER)

    def default(self, component):
        pos = self.read()
        if pos:
            euler = pos.seConfig.euler
            self.data['pan'] = euler.yaw
            self.data['tilt'] = euler.pitch

class PlatineAxisPoster(PocolibsDataStreamInput):
    _type_name = "PLATINE_AXIS_STR"
    _type_url = "http://trac.laas.fr/git/platine-genom/tree/platineStruct.h#n16"

    def initialize(self):
        PocolibsDataStreamInput.initialize(self, PLATINE_AXIS_STR)

    def default(self, component):
        pos = self.read()
        if pos:
            self.data['pan'] = pos.pan
            self.data['tilt'] = pos.tilt

