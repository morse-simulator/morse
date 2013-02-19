import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import PocolibsDataStreamInput
from lwr.struct import *

class LwrPoster(PocolibsDataStreamInput):
    
    _type_name = "Gb_q7"
    _type_url = "http://trac.laas.fr/git/gbM/tree/src/gbStruct.h#n92"

    def initialize(self):
        PocolibsDataStreamInput.initialize(self, Gb_q7)

    def default(self, ci):
        gbm_conf = self.read()
        if gbm_conf:
            self.data['kuka_1'] = gbm_conf.q1
            self.data['kuka_2'] = gbm_conf.q2
            self.data['kuka_3'] = gbm_conf.q3
            self.data['kuka_4'] = gbm_conf.q4
            self.data['kuka_5'] = gbm_conf.q5
            self.data['kuka_6'] = gbm_conf.q6
            self.data['kuka_7'] = gbm_conf.q7

