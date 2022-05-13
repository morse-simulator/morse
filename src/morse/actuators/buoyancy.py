import logging; logger = logging.getLogger("morse." + __name__)
from math import pi, sqrt, isnan
import morse.core.actuator
from morse.core import status
from morse.helpers.components import add_data, add_property
from morse.core import blenderapi
from morse.core.mathutils import *
import numpy as np

__author__     = "David Battle"
__copyright__  = "Copyright 2017, Mission Systems Pty Ltd"
__license__    = "BSD"
__version__    = "1.0.4"
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

    # Returns the fraction of a spherical volume with radius r
    # submerged below the z=0 plane when its centre is at depth d
    def fraction(self,d):
        
        # Sphere radius
        r = self.rad

        if d <= -r:
            return 1.0
        elif d < r and d > -r:
            return 1.0-(0.5+0.75*d/r-0.25*pow(d,3.0)/pow(r,3.0))
        elif d >= r:
            return 0.0

class Buoyancy(morse.core.actuator.Actuator):
    """Write here the general documentation of your actuator.
    It will appear in the generated online documentation.
    """
    _name = "Buoyancy"
    _short_desc = "General purpose buoyancy simulator"

    def __init__(self, obj, parent=None):

        logger.info("%s initialization" % obj.name)

        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        # Get every bge object in the scene
        objs = blenderapi.scene().objects

        # Get the water surface object
        self._water = objs['water']

        # Set the water surface property
        self._water['castable'] = True

        # Initialise all buoyancy spheres in the scene
        self.spheres = [Spheres(child) for child in objs if 'float' in child.name.lower()]

        # Sphere volumes by parent
        sphere_vols = {}

        # Sum sphere volumes
        for s in self.spheres:
            parent = s.obj.parent
            if not parent.name in sphere_vols:
                sphere_vols[parent.name] = s.vol
            else:
                sphere_vols[parent.name] += s.vol

        # Distribute buoyancy among
        # spheres according to volume
        for s in self.spheres:

            # Parent object
            parent = s.obj.parent

            # Parent mass
            mass = parent.mass

            # Parent trim (defaults to zero)
            trim = parent.get('trim',0)

            # The total buoyancy force will equal the weight
            # in air of the flotation elements plus the trim
            mass += trim

            # Buoyancy force when totally submerged
            # Vector points up in the global frame
            buoyancy = -mass * blenderapi.gravity()

            # Buoyancy for this sphere
            s.buoy = buoyancy * s.vol / sphere_vols[parent.name]

        logger.info('Found %d buoyancy elements in scene' % len(self.spheres))
        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):

        # Iterate over spheres
        for s in self.spheres:

            # Current inertial to body transformation
            Ri2b = np.array(s.obj.parent.worldOrientation).T

            # Sphere position in world frame
            pos = s.obj.worldPosition

            # Depth of buoyancy sphere
            depth = pos.z

            if not isnan(depth): 

                up = Vector([0,0,1])

                # Fire a ray down
                target = pos - up 
                _,point,_ = s.obj.rayCast(target, None, 10000, "castable", False, True)

                if not point:

                    # Fire a ray up
                    target = pos + up 
                    _,point,_ = s.obj.rayCast(target, None, 10000, "castable", False, True)

                if point:
                    z = depth - point.z
                else:
                    logger.info("WARNING: Can't find water surface!")
                    z = depth            
                    return

                # Buoyancy vector for sphere in world frame
                Bworld = s.buoy * Spheres.fraction(s,z)

                # Buoyancy in body frame
                Bbody = Ri2b.dot(Bworld)

                # Buoyancy moment in body frame
                Mbody = s.obj.localPosition.cross(Bbody)

                # Buoyancy forces in body frame
                s.obj.parent.applyForce(Bbody,True)

                # Buoyancy moments in body frame
                s.obj.parent.applyTorque(Mbody,True)

                #logger.info('Executed default action')
