import math
import mathutils
#from Mathutils import *

# euler_angle(object) returns a tuple yaw, pitch, roll, in degree,
# corresponding to the transformation between the origin frame and the object
# frame.
#
# It uses the built-in blender function toEuler
def euler_angle(ob):
    rot_matrix = ob.orientation
    matrix = mathutils.Matrix(rot_matrix[0], rot_matrix[1], rot_matrix[2])

    # XXX If we want to have the right handle, we definivly need to transpose
    # the matrix here, but why ?
    matrix.transpose()
    euler = matrix.to_euler()

    return [euler.z, euler.x, euler.y]


# euler_angle(object) returns a tuple yaw, pitch, roll, in degree,
# corresponding to the transformation between the origin frame and the object
# frame.
#
# It is computed "by hand", using the method described in 
#  http://planning.cs.uiuc.edu/node103.html
def euler_angle_old(ob):
    rot_matrix = ob.orientation

    alpha = math.atan2 (rot_matrix[1][0], rot_matrix[0][0])
    beta = math.atan2 (-rot_matrix[2][0], math.sqrt(math.pow(rot_matrix[2][1], 2) + math.pow(rot_matrix[2][2], 2)))
    gamma = math.atan2 (rot_matrix[2][1], rot_matrix[2][2])

    yaw = math.degrees(alpha)
    roll = math.degrees(beta)
    pitch = math.degrees(gamma)

    #### WARNING ####
    # This is probably a temporary solution, while Blender 2.5
    #  is made more stable. Currently it provides a rotation matrix
    #  with opposite signs to that of Blender 2.49b
    #
    # Change the signs of the angles
    #  if the Blender version is 2.5
    yaw = yaw * -1
    pitch = pitch * -1
    roll = roll * -1

    return [yaw, pitch, roll]


def get_rotation_matrix(object):
    """ Return a the rotation matrix of an object.
        Used to transform another object to this one's coordinate system. """
    # Obtain the rotation matrix of the object.
    rot_matrix = object.worldOrientation
    rotation_matrix = mathutils.Matrix(rot_matrix[0], rot_matrix[1], rot_matrix[2])
    # According to the GE documentation, it has to be transposed first
    #rotation_matrix.transpose()

    return rotation_matrix


def invert_rotation_matrix(object):
    """ Return a the inverse of the rotation matrix of an object.
        Used to get the position of another object with respect to
        this one. """
    inverted_matrix = get_rotation_matrix(object)
    inverted_matrix.invert()

    return inverted_matrix


def normalise_angle(angle):
    """ Force the given angle to be between PI and -PI
    
    This function expects an angle given in radians
    It will reduce the input angle to be less than PI,
    and give it the correct sign.
    """
    factor = 1
    # Store the sign of the angle
    if angle < 0.0:
        factor = -1
    # Use positive values to do the conversion
    angle = math.fabs(angle)
    small_angle = angle % math.pi
    division = angle // math.pi
    # Check if the sign should be inversed
    if division % 2 == 1:
        small_angle = math.pi - small_angle
        factor = factor * -1
    angle = factor * small_angle

    return angle


def rotation_direction (current_angle, target_angle, tolerance, speed):
    """ Test the direction in which a rotation should be made

    Using the current angle of a component and the next desired angle.
    Angles are expected in radians """
    # Check which direction to rotate
    if current_angle < (target_angle - tolerance):
        rotation = speed
    elif current_angle > (target_angle + tolerance):
        rotation = -speed
    # If the angle is within the tolerance, don't rotate
    else:
        rotation = 0

    return rotation


def print_matrix_33 (matrix):
    """ print_matrix_33 prints a 3x3 @matrix on stdout """
    for row in matrix:
        line = "[%.4f %.4f %.4f]" % (row[0], row[1], row[2])
        print (line)

def print_vector (vector):
    """ print a vector with 4 decimals per value """
    line = "[%.4f %.4f %.4f]" % (vector[0], vector[1], vector[2])
    print (line)
