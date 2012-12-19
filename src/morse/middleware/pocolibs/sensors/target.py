import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import *
from genPos.struct import *

class TargetPoster(PocolibsDataStreamOutput):
    def __init__(self, component, mw_data):
        super(self.__class__, self).__init__(poster_name(component, mw_data),
                                             GENPOS_TRAJ_POINTS)

        # Initialise the object
        self.obj = GENPOS_TRAJ_POINTS()
        self.obj.numRef = 0

    def write(self, component):
        target_dict = component.local_data['victim_dict']
        self.obj.numRef = self.obj.numRef + 1

        if target_dict:
        # if multiple target, just take the first one
            for target in target_dict.values():
                self.obj.nbPts = 1
                self.obj.points[0].x = target['coordinate']['x']
                self.obj.points[0].y = target['coordinate']['y']
        else:
            self.obj.nbPts = 0

        super(self.__class__, self).write(self.obj)

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    poster = TargetPoster(component_instance, mw_data)
    component_instance.output_functions.append(poster.write)

def write_target(self, component_instance):
    pass
