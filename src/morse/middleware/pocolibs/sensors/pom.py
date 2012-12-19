import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import *
from pom.struct import *

class PomPoster(PocolibsDataStreamOutput):
    def __init__(self, component, mw_data):
        super(self.__class__, self).__init__(poster_name(component, mw_data),
                                             POM_ME_POS)

        # Initialise the object
        self.obj = POM_ME_POS()
        self.obj.kind = POM_ME_ABSOLUTE
        self.obj.confidence = float(component.bge_obj['confidence'])

        # search for the reference poster
        ref_frame = component.bge_obj['reference_frame']
        self.ref_poster = PocolibsDataStreamInput(ref_frame, True, POM_SENSOR_POS)

    def write(self, component):
        ref = self.ref_poster.read()
        if ref:
            self.obj.main.euler.x = component.local_data.get('x', 0.0)
            self.obj.main.euler.y = component.local_data.get('y', 0.0)
            self.obj.main.euler.z = component.local_data.get('z', 0.0)
            self.obj.main.euler.yaw = component.local_data.get('yaw', 0.0)
            self.obj.main.euler.pitch = component.local_data.get('pitch', 0.0)
            self.obj.main.euler.roll = component.local_data.get('roll', 0.0)
            self.obj.date1 = ref.date
            super(self.__class__, self).write(self.obj)

def init_extra_module(self, component_instance, function, mw_data):
    """ Setup the middleware connection with this data

    Prepare the middleware to handle the serialised data as necessary.
    """
    poster = PomPoster(component_instance, mw_data)
    component_instance.output_functions.append(poster.write)

def write_pom(self, component_instance):
    pass
