import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import PocolibsDataStreamInput, poster_name
from genPos.struct import *

class GenPosPoster(PocolibsDataStreamInput):
    _type_name = "GENPOS_CART_SPEED"
    _type_url = "http://trac.laas.fr/git/genPos-genom/tree/genPosStruct.h#n90"
     
    def initialize(self):
        PocolibsDataStreamInput.initialize(self, GENPOS_CART_SPEED)

    def default(self, component):
        speed = self.read()
        if speed:
            self.data['v'] = speed.v
            self.data['w'] = speed.w
