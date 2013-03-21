from morse.core import blenderapi
from morse.core import mathutils
from math import sqrt

class Transformation3d:
    """
    Transformation3d represents a generic 3D transformation. It is used
    by each component of the simulator to know their position in the
    world. Blender does not propose such an interface, only some
    rotation matrix and translation vector.

    Internally, we store an internal 4x4 matrix, and use it to compute
    transformation.  the euler representation is then calculated on base
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
        self.matrix = mathutils.Matrix(([1, 0, 0, 0], \
                                        [0, 1, 0, 0], \
                                        [0, 0, 1, 0], \
                                        [0, 0, 0, 1]))

        self.euler = mathutils.Euler([0, 0, 0])
        if obj != None:
            self.update(obj)

        # For use only by robots moving along the Y axis
        self.correction_matrix = mathutils.Matrix(([0.0, 1.0, 0.0], \
                                                   [-1.0, 0.0, 0.0], \
                                                   [0.0, 0.0, 1.0]))


    @property
    def x(self):
        """
        Return the translation along the x-axis
        """
        if blenderapi.version() < (2, 62, 0):
            return self.matrix[3][0]
        else:
            return self.matrix[0][3]

    @property
    def y(self):
        """
        Return the translation along the y-axis
        """
        if blenderapi.version() < (2, 62, 0):
            return self.matrix[3][1]
        else:
            return self.matrix[1][3]

    @property
    def z(self):
        """
        Return the translation along the z-axis
        """
        if blenderapi.version() < (2, 62, 0):
            return self.matrix[3][2]
        else:
            return self.matrix[2][3]

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

    @property
    def rotation(self):
        """
        Returns the rotation as a unit quaternion
        """
        return self.matrix.to_quaternion()

    @property
    def rotation_matrix(self):
        """
        Returns the rotation as a 3x3 matrix
        """
        return self.matrix.to_3x3()

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
        Update the transformation3D to reflect the transformation
        between obj (a blender object) and the blender world origin
        """
        self.matrix = obj.worldOrientation.to_4x4()

        pos = obj.worldPosition
        for i in range(0, 3):
            if blenderapi.version() < (2, 62, 0):
                self.matrix[3][i] = pos[i]
            else:
                self.matrix[i][3] = pos[i]
        self.matrix[3][3] = 1

        self.euler = self.matrix.to_euler()

    def update_Y_forward(self, obj):
        """
        Update the transformation3D to reflect the transformation
        between obj (a blender object) and the blender world origin.
        In this case, the robot moves forwar along the Y axis.

        Change the values of yaw, pitch, roll for Blender vehicles
        Robots that use the Blender vehicle constraints move in the
        direction of the Y axis, contrary to most of the MORSE components
        that move along the X axis.
        """
        self.matrix = (obj.worldOrientation * self.correction_matrix).to_4x4()

        pos = obj.worldPosition
        for i in range(0, 3):
            if blenderapi.version() < (2, 62, 0):
                self.matrix[3][i] = pos[i]
            else:
                self.matrix[i][3] = pos[i]
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
