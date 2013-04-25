""" This module wraps the calls to the Blender 'mathutils' API. This is intended
for all the cases we need to run MORSE code outside Blender (mostly for
documentation generation purposes).
"""

import sys

# running in Blender?
if not sys.executable.endswith('blender'):
    print("ATTENTION: MORSE is running outside Blender! "
          "(sys.executable != blender)")

    def Matrix(*args):
        return None
    def Vector(*args):
        return None
    def Euler(*args):
        return None
    def Quaternion(*args):
        return None
else:
    from mathutils import Matrix, Vector, Euler, Quaternion

