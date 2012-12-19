import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import PocolibsDataStreamInput, poster_name
from lwr.struct import *

class LwrPoster(PocolibsDataStreamInput):
    def __init__(self, name, delay):
        super(self.__class__, self).__init__(name, delay, Gb_q7)

    def read(self, component):
        gbm_conf = super(self.__class__, self).read()
        if gbm_conf:
            component.local_data['kuka_1'] = gbm_conf.q1
            component.local_data['kuka_2'] = gbm_conf.q2
            component.local_data['kuka_3'] = gbm_conf.q3
            component.local_data['kuka_4'] = gbm_conf.q4
            component.local_data['kuka_5'] = gbm_conf.q5
            component.local_data['kuka_6'] = gbm_conf.q6
            component.local_data['kuka_7'] = gbm_conf.q7

def init_extra_module(self, component, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    poster = LwrPoster(poster_name(component, mw_data), True)
    component.input_functions.append(poster.read)

def read_lwr_config(self, component_instance):
    pass
