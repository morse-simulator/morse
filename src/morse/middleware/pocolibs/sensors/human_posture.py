import logging; logger = logging.getLogger("morse." + __name__)
from morse.middleware.pocolibs_datastream import *
from spark.struct import *

class HumanPoster(PocolibsDataStreamOutput):
    _type_name = "STRUCT_SPARK_CONFIGURATION"
    _type_url = "http://trac.laas.fr/git/spark-genom/tree/sparkStruct.h#n226"

    def initialize(self):
        super(self.__class__, self).initialize(STRUCT_SPARK_CONFIGURATION)

        self.obj = STRUCT_SPARK_CONFIGURATION()
        self.obj.dofNb = 50
        self.obj.changed = 1

    def default(self, component):
        raw_dofs = list(self.data.values())
        #for i in range(self.obj.dofNb):
        for i in range(18):
            self.obj.dof[i] = raw_dofs[i]

        #swap pitch and yaw
        self.obj.dof[5] = raw_dofs[3]
        self.obj.dof[3] = raw_dofs[5]
        #swap right shoulder: X <- Y, Y <- Z, Z <- X
        self.obj.dof[18] = raw_dofs[19]
        self.obj.dof[19] = raw_dofs[20]
        self.obj.dof[20] = raw_dofs[18]
        #Right arm length
        self.obj.dof[21] = 0
        self.obj.dof[22] = raw_dofs[21]
        self.obj.dof[23] = 0

        for i in range(24, 27):
            self.obj.dof[i] = raw_dofs[i-2]
        #left shoulder:
        #X <- Y
        self.obj.dof[27] = -raw_dofs[26]
        #Y <- Z
        self.obj.dof[28] = raw_dofs[27]
        #Z <- X
        self.obj.dof[29] = raw_dofs[25]

        self.obj.dof[30] = 0
        self.obj.dof[31] = raw_dofs[28]
        self.obj.dof[32] = 0

        for i in range(33, self.obj.dofNb):
                self.obj.dof[i] = raw_dofs[i-4]
        self.write(self.obj)

