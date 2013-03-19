import logging; logger = logging.getLogger("morse." + __name__)
import math

# euler_angle(object) returns a tuple yaw, pitch, roll, in degree,
# corresponding to the transformation between the origin frame
# and the object frame.
#
# It uses the built-in blender function to_euler
def euler_angle(ob):
    rot_matrix = ob.worldOrientation

    # XXX If we want to have the right handle, we definivly need to transpose
    # the matrix here, but why ?
    rot_matrix.transpose()
    euler = rot_matrix.to_euler()

    return [euler.z, euler.x, euler.y]


def get_rotation_matrix(object):
    """ Return a the rotation matrix of an object.
    Used to transform another object to this one's coordinate system. """

    import copy
    return copy.copy(object.worldOrientation)

def invert_rotation_matrix(object):
    """ Return the inverse of the rotation matrix of an object.

    Used to get the position of another object with respect to
    this one. """
    rotation_matrix = get_rotation_matrix(object)

    # Return a new inverted matrix (requires Blender 2.58)
    return rotation_matrix.inverted()


def normalise_angle(angle):
    """ Force the given angle to be between PI and -PI
    
    This function expects an angle given in radians
    It will reduce the input angle to be less than PI,
    and give it the correct sign.

    Using new method proposed by David Hodo:
    hododav@tigermail.auburn.edu
    """
    return ((angle+math.pi)%(2*math.pi))-math.pi


def rotation_direction(current_angle, target_angle, tolerance, speed):
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


def print_matrix_33(matrix):
    """ print_matrix_33 prints a 3x3 @matrix on stdout """
    for row in matrix:
        line = "[% .4f % .4f % .4f]" % (row[0], row[1], row[2])
        logger.info(line)

def print_matrix_44(matrix):
    """ print_matrix_44 prints a 4x4 @matrix on stdout """
    for row in matrix:
        line = "[% .4f % .4f % .4f % .4f]" % (row[0], row[1], row[2], row[3])
        logger.info(line)

def print_vector (vector):
    """ print a vector with 4 decimals per value """
    line = "[% .4f % .4f % .4f]" % (vector[0], vector[1], vector[2])
    logger.info(line)


def fill_vector(vector, point):
    """ Copy the contents of a list into an existing vector structure. """
    for i in range(3):
        vector[i] = point[i]
