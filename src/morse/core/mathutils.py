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

def Matrix(matrix=None):
    if not fake:
        if matrix:
            return mathutils.Matrix(matrix)
        else:
            return mathutils.Matrix()
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

