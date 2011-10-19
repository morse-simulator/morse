import GameLogic
import mathutils
from math import sqrt

class Transformation3d:
    """
    Transformation3d represents a generic 3D transformation. It is used
    by each component of the simulator to know their position in the
    world. Blender does not propose such an interface, only some
    rotation matrix and translation vector.

    Internaly, we store an internal 4x4 matrix, and use it to compute
    transformation.  the euler representation is then calculed on base
    of matrix (euler ZYX convention)

    Note : Blender store its matrix in column major mode ...
    """

    def __init__(self, obj):
        """
        Construct a transformation3d. Generate the identify
        transformation if no object is associated to it. Otherwise,
        returns the transformation3D between this object and the world
        reference

        """
        if GameLogic.blenderVersion <= (2,56,0):
        #if GameLogic.pythonVersion <= (3,1,0):
            self.matrix = mathutils.Matrix([1, 0, 0, 0], \
                                           [0, 1, 0, 0], \
                                           [0, 0, 1, 0], \
                                           [0, 0, 0, 1])
        else:
            self.matrix = mathutils.Matrix(([1, 0, 0, 0], \
                                            [0, 1, 0, 0], \
                                            [0, 0, 1, 0], \
                                            [0, 0, 0, 1]))

        self.euler = mathutils.Euler([0, 0, 0])
        if obj != None:
            self.update(obj)

    @property
    def x(self):
        """
        Return the translation  against the x axle
        """
        return self.matrix[3][0]

    @property
    def y(self):
        """
        Return the translation  against the y axle
        """
        return self.matrix[3][1]

    @property
    def z(self):
        """
        Return the translation  against the z axle
        """
        return self.matrix[3][2]

    @property
    def yaw(self):
        """
        Returns Euler Z axis, in radian
        """
        return self.euler.z

    @property
    def pitch(self):
        """
        Returns Euler Y axis, in radian
        """
        return self.euler.y

    @property
    def roll(self):
        """
        Returns Euler X axis, in radian
        """
        return self.euler.x

    def transformation3d_with(self, t3d):
        """
        Compute the transformation between itself and another
        transformation t3d. In other words, A.transformation3d_with(B)
        returns inv(A) * B.

        self is not modified by the call of this function
        """
        res = Transformation3d(None)
        o2m = self.matrix.copy()
        o2m.invert()
        res.matrix = o2m * t3d.matrix
        res.euler = res.matrix.to_euler()
        return res

    def distance(self, t3d):
        """ 
        Compute the 3d distance between two transformations. 

        nor self, nor t3d are modified by the call of this method
        """
        diff_x = self.x - t3d.x
        diff_y = self.y - t3d.y
        diff_z = self.z - t3d.z

        return sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z)

    def distance_2d(self, t3d):
        """ 
        Compute the 2d distance between two transformations. 

        nor self, nor t3d are modified by the call of this method
        """
        diff_x = self.x - t3d.x
        diff_y = self.y - t3d.y

        return sqrt(diff_x * diff_x + diff_y * diff_y)

    def update(self, obj):
        """
        Update the transformation3D to reflect the tranformation
        between ob (a blender object) and the blender world origin

        """
        rot_matrix = obj.orientation
        if GameLogic.blenderVersion <= (2,56,0):
        #if GameLogic.pythonVersion <= (3,1,0):
            self.matrix = mathutils.Matrix(rot_matrix[0], rot_matrix[1], \
                                                          rot_matrix[2])
            self.matrix.resize4x4()
        else:
            self.matrix = mathutils.Matrix((rot_matrix[0], rot_matrix[1], \
                                                          rot_matrix[2]))
            self.matrix.resize_4x4()

        pos = obj.worldPosition
        for i in range(0, 3):
            self.matrix[3][i] = pos[i]
        self.matrix[3][3] = 1

        self.euler = self.matrix.to_euler()

    def __str__(self):
        """
        String representation of a transformation3D
        """
        res = "x : " + str(self.x)
        res += " y : " + str(self.y)
        res += " z : " + str(self.z)
        res += " yaw : " + str(self.yaw)
        res += " pitch : " + str(self.pitch)
        res += " roll : " + str(self.roll)
        return res
