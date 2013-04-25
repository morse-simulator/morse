""" This module wraps the calls to the Blender 'mathutils' API. This is intended
for all the cases we need to run MORSE code outside Blender (mostly for
documentation generation purposes).
"""

import sys

fake = False

# running in Blender?
if sys.executable.endswith('blender'):
    import mathutils
else:
    print("ATTENTION: MORSE is running outside Blender! (sys.executable != blender)")
    fake = True


class Matrix:
    def __init__(self, matrix=None):
        if not fake:
            if matrix:
                self = mathutils.Matrix(matrix)
            else:
                self = mathutils.Matrix()
        else:
            self = None

    @staticmethod
    def Rotation(x, y, z):
        if not fake:
            return mathutils.Matrix.Rotation(x, y, z)
        else:
            return None

def Vector(vector):
    if not fake:
        return mathutils.Vector(vector)
    else:
        return None

def Euler(angle):
    if not fake:
        return mathutils.Euler(angle)
    else:
        return None

def Quaternion(axis=None, angle=None):
    if not fake:
        if axis and angle:
            return mathutils.Quaternion(axis, angle)
        else:
            return mathutils.Quaternion()
    else:
        return None
