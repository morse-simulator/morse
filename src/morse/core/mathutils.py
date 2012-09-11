""" This module wraps the calls to the Blender 'mathutils' API. This is intended
for all the cases we need to run MORSE code outside Blender (mostly for
documentation generation purposes).
"""

import morse

fake = False

if morse.running_in_blender:
    import mathutils
else:
    fake = True

def Matrix(matrix):
    if not fake:
        return mathutils.Matrix(matrix)
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

