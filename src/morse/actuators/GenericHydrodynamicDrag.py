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

import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.actuator

from morse.core.services import service, async_service, interruptible
from morse.core import status
from morse.helpers.components import add_data, add_property
from morse.core import blenderapi
from morse.core.mathutils import *
import numpy as np

__author__     = "David Battle" 
__copyright__  = "Copyright 2018, Mission Systems Pty Ltd"
__license__    = "BSD"
__version__    = "1.0.0"
__maintainer__ = "David Battle"
__email__      = "david.battle@missionsystems.com.au"
__status__     = "Production"

class Spheres(object):

    def __init__(self,object):

        # Mean radius of object
        def meanRadius(self):

            # Assume one mesh
            mesh = object.meshes[0]
            scale = object.scaling

            sum = 0.0
            nVertices = mesh.getVertexArrayLength(0)

            # Iterate through vertices
            for v in range(nVertices):
                 vertex = mesh.getVertex(0,v)

                 # Sum distances from origin
                 sum += sqrt((scale.x*vertex.x)**2 +
                             (scale.y*vertex.y)**2 +
                             (scale.z*vertex.z)**2)

            # Return mean distance
            return sum / nVertices

        # Useful properties of spheres
        self.rad = meanRadius(object)
        self.vol = 4*pi*self.rad**3
        self.obj = object
        self.buoy = 0.0

def GetDimensions(object = None, roundit = 3, offset = 1, meshnum = 0):

    """
    Gets the dimensions of the object (what you see under dimensions in the properties window in the 3D menu).
    mesh = which mesh to use to get the object's dimensions.
    roundit = how far down to round the returned dimension values; set it to a negative number to not round the numbers off at all.
    offset = Whether or not to return the offset point of the dimensions (the center point);
    This negated (-offset, literally) is the origin point, generally.
    meshnum = The index of the mesh to use. Usually 0 is okay.
    """

    if object == None:
        object = logic.getCurrentController().owner

    s = object.worldScale

    mesh = object.meshes[meshnum]

    #print (dir(mesh))

    verts = [[], [], []]

    originpos = [0, 0, 0]

    for mat in range(len(mesh.materials)):

        for v in range(mesh.getVertexArrayLength(mat)):

            vert = mesh.getVertex(mat, v)

            pos = vert.getXYZ()

            verts[0].append(pos[0])
            verts[1].append(pos[1])
            verts[2].append(pos[2])

    verts[0].sort()
    verts[1].sort()
    verts[2].sort()

    if offset != 0:

        offsetpos = [
            (verts[0][len(verts[0])-1] + verts[0][0]) / 2,
            (verts[1][len(verts[1])-1] + verts[1][0]) / 2,
            (verts[2][len(verts[2])-1] + verts[2][0]) / 2,
            ]

    if roundit &gt != 0:
        size = [
        round( (verts[0][len(verts[0]) - 1] - verts[0][0]) * s[0] , roundit),
        round( (verts[1][len(verts[0]) - 1] - verts[1][0]) * s[1] , roundit),
        round( (verts[2][len(verts[0]) - 1] - verts[2][0]) * s[2] , roundit) ]
    else:
        size = [(verts[0][len(verts[0]) - 1] - verts[0][0]) * s[0],
        (verts[1][len(verts[0]) - 1] - verts[1][0]) * s[1],
        (verts[2][len(verts[0]) - 1] - verts[2][0]) * s[2]]

    #originpos = [0, 0, 0]

    if offset:
        return (mathutils.Vector(size), mathutils.Vector(offsetpos))

    else:
        return (mathutils.Vector(size), None)

class GenericHydrodynamicDrag(morse.core.actuator.Actuator): 
    """Write here the general documentation of your actuator.
    It will appear in the generated online documentation.
    """
    _name = "Drag"
    _short_desc = "Simple hydrodynamic drag simulator for submerged objects"

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        # Density of seawater
        seaWaterRho = 1025

        # Get every bge object in the scene
        objs = blenderapi.scene().objects
 
        # Select objects with drag property
        self.dragobjs = [o for o in objs if 'drag' in o]

        dl = 5
        da = 0.25

        # Linear damping coefficients
        self.Dl = np.array([dl, dl, dl, da, da, da])

        logger.info('Component initialized')

    def default_action(self):

        # Iterate over objects
        for obj in self.dragobjs:

            # Position in world frame
            pos = obj.worldPosition

            # Submerged?
            if pos.z < 0:

                # Linear velocities in body frame
                linVel = obj.localLinearVelocity

                # Angular velocities in body frame
                angVel = obj.localAngularVelocity

                # Body-frame linear velocity components
                u = linVel.x # Surge
                v = linVel.y # Sway
                w = linVel.z # Heave

                # Body-frame angular velocity components
                p = angVel.x # Roll
                q = angVel.y # Pitch
                r = angVel.z # Yaw

                # # Velocity vector
                vel = np.array([u,v,w,p,q,r])

                dl = 30
                da = 0.5

                # Quadratic damping matrix
                Dn = np.array([dl*abs(u),
                               dl*abs(v),
                               dl*abs(w),
                               da*abs(p),
                               da*abs(q),
                               da*abs(r)])

                # Linear and quadratic damping
                Tau = -(self.Dl + Dn) * vel

                #----------------------------------------------------------
                # Apply forces and moments in body frame

                # Hydrodynamic forces in body frame
                obj.applyForce(Tau[:3:],True)

                # Hydrodynamic moments in body frame
                obj.applyTorque(Tau[3::],True)
