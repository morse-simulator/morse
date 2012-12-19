import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import PocolibsDataStreamInput, poster_name
from genPos.struct import *

class GenPosPoster(PocolibsDataStreamInput):
    def __init__(self, name, delay):
        super(self.__class__, self).__init__(name, delay, GENPOS_CART_SPEED)

    def read(self, component):
        speed = super(self.__class__, self).read()
        if speed:
            component.local_data['v'] = speed.v
            component.local_data['w'] = speed.w

def init_extra_module(self, component, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """

    poster = GenPosPoster(poster_name(component, mw_data), True)
    component.input_functions.append(poster.read)

def read_genpos():
    pass
