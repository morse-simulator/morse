import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import *
from platine.struct import *
from math import degrees

class PlatinePoster(PocolibsDataStreamOutput):
    def __init__(self, component, mw_data):
        super(self.__class__, self).__init__(poster_name(component, mw_data),
                                             PLATINE_STATES)
        self.obj = PLATINE_STATES()

    def write(self, component):
        self.obj.stateRad.pos.pan = component.local_data['pan']
        self.obj.stateRad.pos.tilt = component.local_data['tilt']
        self.obj.stateDeg.pos.pan = degrees(component.local_data['pan'])
        self.obj.stateDeg.pos.tilt = degrees(component.local_data['tilt'])
        super(self.__class__, self).write(self.obj)

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    poster = PlatinePoster(component_instance, mw_data)
    component_instance.output_functions.append(poster.write)

def write_platine_posture(self, component_instance):
    pass
