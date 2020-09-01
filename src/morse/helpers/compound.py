# ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ Mission Systems Pty Ltd ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Project: WamV-Morse-Sim
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Primary Author:
# david battle <david.battle@missionsystems.com.au>
# Other Author(s):
# none
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Date Created:
# 29/01/2019
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

from morse.core.mathutils import *
from morse.builder import bpymorse

# Some functions to translate and rotate compound robots.
# Compound robots do not have a parent object in the Blender
# sense, but are held together by rigid body constraints.
# Without special treatment, they fall apart when moved.

__author__     = "David Battle"
__copyright__  = "Copyright 2017, Mission Systems Pty Ltd"
__license__    = "GPL"
__version__    = "1.0.0"
__maintainer__ = "David Battle"
__email__      = "david.battle@missionsystems.com.au"
__status__     = "Production"

# Find all objects in the scene that
# are constrained to the current object
def find_family(self):

    objs = bpymorse.get_objects()

    # Initialise family to parent
    family =[self.name]

    while True:

        more = 0

        for f in family:
            remaining = [o for o in objs if o.name not in family]
            for r in remaining:

                # Check forward constraints
                for c in objs[f].constraints:
                    if 'target' in dir(c):
                        if c.target.name == r.name:
                            family.append(r.name)
                            more += 1

                # Check backward constraints
                for c in r.constraints:
                    if 'target' in dir(c):
                        if c.target.name == f:
                            family.append(r.name)
                            more += 1
        if not more:
            break

    return set(family)
    
def translate_compound(self, x=0.0, y=0.0, z=0.0):

    objs = bpymorse.get_objects()
    family = find_family(self)

    # print("found %d objects constrained by %s:" % (len(family)-1, self.name))
    # print([f for f in family if f != self.name])

    for f in family:
        oldl = objs[f].location
        objs[f].location = (oldl.x+x,oldl.y+y,oldl.z+z)  

def rotate_compound(self, x=0.0, y=0.0, z=0.0):

    objs = bpymorse.get_objects()
    family = find_family(self)

    # Parent origin
    origin = self.location

    # Rotation matrix
    rot = Euler([x,y,z]).to_matrix()

    for f in family:
        oldl = objs[f].location
        oldr = objs[f].rotation_euler

        # Rotate each object around the parent origin
        objs[f].rotation_euler = (oldr.x+x,oldr.y+y,oldr.z+z)
        objs[f].location = rot * (oldl - origin) + origin
