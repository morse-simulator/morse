from morse.core import mathutils
from math import cos, sin

def linear_velocities(prev, now, dt):
    """
    Return angular velocities between two poses

    :param prev: the precedent pose, as a Transformation3d
    :param now: the current pose, as a Transformation3d
    :param dt: time elapsed between the two poses acquisition, in sec
    """
    return (now.translation - prev.translation) / dt

def angular_velocities(prev, now, dt):
    """
    Return angular velocities between two poses

    :param prev: the precedent pose, as a Transformation3d
    :param now: the current pose, as a Transformation3d
    :param dt: time elapsed between the two poses acquisition, in sec


    See https://www.astro.rug.nl/software/kapteyn/_downloads/attitude.pdf
    for equation description
    """
    patt = mathutils.Vector(prev.euler)
    att = mathutils.Vector(now.euler)
    euler_rate = (att - patt) /dt

    c0 = cos(att[0])
    c1 = cos(att[1])
    s0 = sin(att[0])
    s1 = sin(att[1])

    m = mathutils.Matrix(([1, 0, -s1],
                          [0, c0, c1 * s0],
                          [0, -s0, c1 * c0]))

    return m * euler_rate
