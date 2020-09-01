import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.actuator

from morse.core.services import service, async_service, interruptible
from morse.core import status
from morse.helpers.components import add_data, add_property
from morse.core import blenderapi
from morse.core.mathutils import *
from morse.builder import bpymorse
from math import isnan
import numpy as np

__author__     = "David Battle"
__copyright__  = "Copyright 2018, Mission Systems Pty Ltd"
__license__    = "BSD"
__version__    = "1.1.0"
__maintainer__ = "David Battle"
__email__      = "david.battle@missionsystems.com.au"
__status__     = "Production"

class dragBoundary(object):

    def __init__(self,object):

        def get_dims(obj):

            mesh = obj.meshes[0]
            collection = [[], [], []]
            for mat_index in range(mesh.numMaterials):
                for vert_index in range(mesh.getVertexArrayLength(mat_index)):
                    vert_XYZ = mesh.getVertex(mat_index, vert_index).XYZ
                    [collection[i].append(vert_XYZ[i]) for i in range(3)]
            return Vector([abs(max(axis)) + abs(min(axis)) for axis in collection])

        # Some useful properties
        dims = get_dims(object)

        # Frontal areas
        self.Fx = dims.y * dims.z
        self.Fy = dims.x * dims.z
        self.Fz = dims.x * dims.y

        # Tangential areas
        self.Tx = 2.0 * dims.x * (dims.y + dims.z)
        self.Ty = 2.0 * dims.y * (dims.x + dims.z)
        self.Tz = 2.0 * dims.z * (dims.x + dims.y)

        self.dims = dims
        self.obj  = object
   
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

        # Get bge objects in scene
        bge_objs = blenderapi.scene().objects

        # All bpy objects in scene
        bpy_objs = bpymorse.get_objects()

        # Find all objects with drag properties
        self.boundaries = [dragBoundary(bge_objs[obj.name]) for obj in bpy_objs if 'drag' in obj.game.properties]

        # Get current object
        try:
            self.current = bge_objs['fake.current']
        except:
            self.current = None

        logger.info('Found %d drag elements in scene' % len(self.boundaries))
        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):

        if self.current:
            # Current vector in world frame 
            current_world = self.current['vec']
        else:
            current_world = Vector([0,0,0])

        # Iterate over objects
        for box in self.boundaries:

            # Position in world frame
            pos = box.obj.worldPosition

            # Depth of drag box
            depth = pos.z

            if not isnan(depth): 

                up = Vector([0,0,1])

                # Fire a ray down
                target = pos - up 
                _,point,_ = box.obj.rayCast(target, None, 10000, "castable", False, True)

                if not point:

                    # Fire a ray up
                    target = pos + up 
                    _,point,_ = box.obj.rayCast(target, None, 10000, "castable", False, True)

                if point:
                    z = depth - point.z
                else:
                    logger.info("WARNING: Can't find water surface!")
                    z = depth            

                # Submerged?
                if z < 0:
                    density = 1025 # water
                else:
                    density = 50 # air

                # Matrix inverse for this drag box
                box_inv = box.obj.localOrientation.transposed()

                # Wind vector relative to drag box (ENU)
                current_rel = box_inv * current_world

                # Linear velocities in body frame
                linVel = box.obj.localLinearVelocity

                # Effective velocity including wind 
                effVel = linVel - current_rel

                # Angular velocities in body frame
                angVel = box.obj.localAngularVelocity

                # Body-frame linear velocity components
                u = effVel.x # Surge
                v = effVel.y # Sway
                w = effVel.z # Heave

                # Body-frame angular velocity components
                p = angVel.x # Roll
                q = angVel.y # Pitch
                r = angVel.z # Yaw

                # # Velocity vector 
                vel = np.array([u,v,w,p,q,r])

                # Quadratic damping
                Dn = np.array([1e-1 * density * box.Fx * abs(u),
                               1e-1 * density * box.Fy * abs(v),
                               1e-1 * density * box.Fz * abs(w),
                               1e-6 * density * box.Tx * abs(p),
                               1e-6 * density * box.Fz * abs(q),
                               1e-6 * density * box.Fy * abs(r)])

                # Linear and quadratic damping
                Tau = -Dn * vel

                #----------------------------------------------------------
                # Apply forces and moments in body frame

                # Hydrodynamic forces in body frame
                box.obj.applyForce(Tau[:3],True)

                # Hydrodynamic moments in body frame - seems to create instability!
                box.obj.applyTorque(Tau[3:],True)
