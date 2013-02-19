import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import *
from pom.struct import *

class PomSensorPosPoster(PocolibsDataStreamInput):
    def initialize(self):
        PocolibsDataStreamInput.initialize(self, POM_SENSOR_POS)

class PomSensorPoster(PocolibsDataStreamOutput):
    _type_name = "POM_ME_POS"
    _type_url = "http://trac.laas.fr/git/pom-genom/tree/pomStruct.h#n180"

    def initialize(self):
        super(self.__class__, self).initialize(POM_ME_POS)

        # Initialise the object
        self.obj = POM_ME_POS()
        self.obj.kind = POM_ME_ABSOLUTE
        self.obj.confidence = float(self.component_instance.bge_object['confidence'])

        # search for the reference poster
        ref_frame = self.component_instance.bge_object['reference_frame']
        self.ref_poster = PomSensorPosPoster(self.component_instance, { 'poster': ref_frame })

    def default(self, ci):
        ref = self.ref_poster.read()
        if ref:
            self.obj.main.euler.x = self.data.get('x', 0.0)
            self.obj.main.euler.y = self.data.get('y', 0.0)
            self.obj.main.euler.z = self.data.get('z', 0.0)
            self.obj.main.euler.yaw = self.data.get('yaw', 0.0)
            self.obj.main.euler.pitch = self.data.get('pitch', 0.0)
            self.obj.main.euler.roll = self.data.get('roll', 0.0)
            self.obj.date1 = ref.date
            self.write(self.obj)

class PomPoster(PocolibsDataStreamOutput):
    _type_name = "POM_POS"
    _type_url = "http://trac.laas.fr/git/pom-genom/tree/pomStruct.h#n211"

    def initialize(self):
        super(self.__class__, self).initialize(POM_POS)

        # Initialise the object
        self.obj = POM_POS()
        self.obj.date = 0
        self.obj.pomTickDate = 0

    def default(self, ci):
        self.obj.mainToOrigin.euler.x = self.data.get('x', 0.0)
        self.obj.mainToOrigin.euler.y = self.data.get('y', 0.0)
        self.obj.mainToOrigin.euler.z = self.data.get('z', 0.0)
        self.obj.mainToOrigin.euler.yaw = self.data.get('yaw', 0.0)
        self.obj.mainToOrigin.euler.pitch = self.data.get('pitch', 0.0)
        self.obj.mainToOrigin.euler.roll = self.data.get('roll', 0.0)
        self.obj.date = self.obj.date + 1
        self.obj.pomTickDate = self.obj.date
        self.write(self.obj)

