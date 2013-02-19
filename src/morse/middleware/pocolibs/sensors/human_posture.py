import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import *
from spark.struct import *

class HumanPoster(PocolibsDataStreamOutput):
    _type_name = "SPARK_CONFIGURATION"
    _type_url = "http://trac.laas.fr/git/spark-genom/tree/sparkStruct.h#n226"

    def initialize(self):
        super(self.__class__, self).initialize(SPARK_CONFIGURATION)

        self.obj = SPARK_CONFIGURATION()
        self.obj.dofNb = 46
        self.obj.changed = 1

    def default(self, component):
        raw_dofs = list(self.data.values())
        for i in range(self.obj.dofNb):
            self.obj.dof[i] = raw_dofs[i]
        self.write(self.obj)

