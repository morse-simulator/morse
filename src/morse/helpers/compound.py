# ~ ~ ~ ~ ~ ~ ~ ~ ~ ~ Mission Systems Pty Ltd ~ ~ ~ ~ ~ ~ ~ ~ ~ ~
# This file is subject to the terms and conditions defined in
# file 'LICENSE', which is part of this source code package.
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Project: WamV-Morse-Sim
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Primary Author:
# david battle <david.battle@missionsystems.com.au>
# Other Author(s):
# Aspen Eyers
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Date Created:
# 29/01/2019
# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

from morse.core.mathutils import *
from morse.builder import bpymorse
import numpy as np 
import bpy
from scipy.spatial.transform import Rotation

# Some functions to translate and rotate compound robots.
# Compound robots do not have a parent object in the Blender
# sense, but are held together by rigid body constraints.
# Without special treatment, they fall apart when moved.

__author__     = "David Battle"
__author__     = "Aspen Eyers"
__copyright__  = "Copyright 2017, Mission Systems Pty Ltd"
__license__    = "GPL"
__version__    = "1.0.2"
__maintainer__ = "David Battle"
__email__      = "david.battle@missionsystems.com.au"
__email__      = "aspen.eyers@missionsystems.com.au"
__status__     = "Production"

# Find all objects in the scene that
# are constrained to the current object
def find_family(self):

    # Initialise family to parent
    objs = bpymorse.get_objects()
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
    for f in family:
        print(f)
        oldl = objs[f].location
        objs[f].location = (oldl.x+x,oldl.y+y,oldl.z+z)  
        # update needed so that matrix_world will update
        # print(objs[f].matrix_world)
        bpy.context.scene.update()

    return

def rotate_compound(self, x=0.0, y=0.0, z=0.0):
    transform_compound_local(self, roll=x, pitch=y, yaw=z)
    return

def transform_compound_local(self, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0):
    objs = bpymorse.get_objects()
    family = find_family(self)
    parent = objs[self.name]
    
    TF = get_tf(x,y,z,roll,pitch,yaw)
    world2parent = np.matrix( parent.matrix_world, dtype="double" )
        
    for f in family:
        obj = objs[f]
        current_pose = np.matrix( obj.matrix_world, dtype="double" )
    
        # Find TF in parent frame   
        # transform = np.dot(world2parent, np.dot(TF, np.linalg.inv(world2parent)) )         
        transform = np.dot(np.linalg.inv(world2parent), np.dot(TF, world2parent) )         
            
        new_pose = np.dot(transform, current_pose)
        obj.matrix_world = Matrix( new_pose.tolist() )
    return

def get_tf(dx=0.0,dy=0.0,dz=0.0,droll=0.0,dpitch=0.0,dyaw=0.0):
    rot = Rotation.from_euler("zyx", [dyaw,dpitch,droll])
    rotmat = rot.as_matrix() # Must have scipy version >= 1.4.0
    TF = np.matrix([[1.0,0.0,0.0,dx],
                    [0.0,1.0,0.0,dy],
                    [0.0,0.0,1.0,dz],
                    [0.0,0.0,0.0,1.0]], dtype="double")
    TF[0:3,0:3] = rotmat[0:3,0:3]
    return TF


