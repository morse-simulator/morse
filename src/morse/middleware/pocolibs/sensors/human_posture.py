import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import *
from spark.struct import *

class HumanPoster(PocolibsDataStreamOutput):
    _type_name = "STRUCT_SPARK_CONFIGURATION"
    _type_url = "http://trac.laas.fr/git/spark-genom/tree/sparkStruct.h#n226"

    def initialize(self):
        super(self.__class__, self).initialize(STRUCT_SPARK_CONFIGURATION)

        self.obj = STRUCT_SPARK_CONFIGURATION()
        self.obj.dofNb = 46
        self.obj.changed = 1

    def default(self, component):
        raw_dofs = list(self.data.values())
        #for i in range(self.obj.dofNb):
        for i in range(21):
            self.obj.dof[i] = raw_dofs[i]

            #swap pitch and yaw
            self.obj.dof[5] = raw_dofs[3]
            self.obj.dof[3] = raw_dofs[5]
            #Right arm length 
            self.obj.dof[21] = 0
            self.obj.dof[22] = raw_dofs[21]
            self.obj.dof[23] = 0

            for i in range(24, 30):
                self.obj.dof[i] = raw_dofs[i-2]

            self.obj.dof[30] = 0
            self.obj.dof[31] = raw_dofs[28]
            self.obj.dof[32] = 0
            for i in range(33, self.obj.dofNb):
                self.obj.dof[i] = raw_dofs[i-4]

        self.write(self.obj)

