import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import *
from spark.struct import *

class HumanPoster(PocolibsDataStreamOutput):
    def __init__(self, component, mw_data):
        super(self.__class__, self).__init__(poster_name(component, mw_data),
                                             SPARK_CONFIGURATION)
        self.obj = SPARK_CONFIGURATION()
        self.obj.dofNb = 46
        self.obj.changed = 1

    def write(self, component):
        raw_dofs = list(component.local_data.values())
        for i in range(self.obj.dofNb):
            self.obj.dof[i] = raw_dofs[i]
        super(self.__class__, self).write(self.obj)


def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    poster = PlatinePoster(component_instance, mw_data)
    component_instance.output_functions.append(poster.write)

def export_posture(self, component_instance):
    pass
