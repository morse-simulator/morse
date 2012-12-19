from morse.middleware.pocolibs_datastream import PocolibsDataStreamInput, poster_name
from pom.struct import *
from platine.struct import *

class PlatinePoster(PocolibsDataStreamInput):
    def __init__(self, name, delay):
        super(self.__class__, self).__init__(name, delay, POM_SE_POSTER)

    def read(self, component):
        pos = super(self.__class__, self).read()
        if pos:
            euler = pos.seConfig.euler
            component.local_data['pan'] = euler.yaw
            component.local_data['tilt'] = euler.pitch

class PlatineAxisPoster(PocolibsDataStreamInput):
    def __init__(self, name, delay):
        super(self.__class__, self).__init__(name, delay, PLATINE_AXIS_STR)

    def read(self, component):
        pos = super(self.__class__, self).read()
        if pos:
            component.local_data['pan'] = pos.pan
            component.local_data['tilt'] = pos.tilt

def init_extra_module(self, component, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """

    if mw_data[1] == "read_platine":
        poster = PlatinePoster(poster_name(component, mw_data), True)
    elif mw_data[1] == "read_platine_axis":
        poster = PlatineAxisPoster(poster_name(component, mw_data), True)
    else:
        poster = None
    component.input_functions.append(poster.read)

def read_platine():
    pass

def read_platine_axis():
    pass
