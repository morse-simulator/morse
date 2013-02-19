import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import *
from genPos.struct import *

class TargetPoster(PocolibsDataStreamOutput):
    _type_name = "GENPOS_TRAJ_POINTS"
    _type_url = "http://trac.laas.fr/git/genPos-genom/tree/genPosStruct.h#n144"

    def initialize(self):
        super(self.__class__, self).initialize(GENPOS_TRAJ_POINTS)

        # Initialise the object
        self.obj = GENPOS_TRAJ_POINTS()
        self.obj.numRef = 0

    def default(self, ci):
        target_dict = self.data['victim_dict']
        self.obj.numRef = self.obj.numRef + 1

        if target_dict:
        # if multiple target, just take the first one
            for target in target_dict.values():
                self.obj.nbPts = 1
                self.obj.points[0].x = target['coordinate']['x']
                self.obj.points[0].y = target['coordinate']['y']
        else:
            self.obj.nbPts = 0

        self.write(self.obj)

