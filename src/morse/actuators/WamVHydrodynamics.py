import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.actuator
from morse.helpers.components import add_data, add_property
from math import degrees, radians, pi
from morse.core import blenderapi
from morse.core.mathutils import *
from morse.helpers.compound import *
import numpy as np

__author__     = "David Battle"
__copyright__  = "Copyright 2018, Mission Systems Pty Ltd"
__license__    = "BSD"
__version__    = "1.0.0"
__maintainer__ = "David Battle"
__email__      = "david.battle@missionsystems.com.au"
__status__     = "Production"

# The following calculates approximate hydrodynamic
# forces for each WAM-V hull separately.
# The model follows Fossen's matrix treatment with the
# addition of some fin lift terms, without which the
# WAM-V appears to have virtually no directional stability.
# For now, calculations are in the Blender coordinate frame.

def do_model(self,bge_object):

    ####################################
    # Check if drag element is submerged
    # Object position in world frame
    # pos = bge_object.worldPosition

    # # Depth of object (assuming water level = 0)
    # depth = pos.z

    # up = Vector([0,0,1])

    # # Fire a ray down
    # target = pos - up 
    # _,point,_ = bge_object.rayCast(target, None, 10000, "castable", False, True)

    # if not point:

    #     # Fire a ray up
    #     target = pos + up 
    #     _,point,_ = bge_object.rayCast(target, None, 10000, "castable", False, True)

    # if point:
    #     z = depth - point.z
    # else:
    #     logger.info("WARNING: Can't find water surface!")
    #     z = depth

    # # Skip if not submerged
    # if z > 1:
    #     return           

    if self.current:
        # Current vector in world frame 
        current_world = self.current['vec']
    else:
        current_world = Vector([0,0,0])

    ####################################

   # Orientation matrix in world frame
    hull_mat = bge_object.worldOrientation

    # Matrix inverse
    hull_inv = hull_mat.transposed()

    # Current vector relative to glider        
    current_rel = hull_inv * current_world

    # Linear velocities in body frame
    linVel = bge_object.localLinearVelocity - current_rel

    # Angular velocities in body frame
    angVel = bge_object.localAngularVelocity

    # Body-frame linear velocity components
    u = linVel.x # Surge
    v = linVel.y # Sway
    w = linVel.z # Heave

    # Body-frame angular velocity components
    p = angVel.x # Roll
    q = angVel.y # Pitch
    r = angVel.z # Yaw

    # Velocity vector 
    vel = np.array([u,v,w,p,q,r])

    # Quadratic damping matrix
    Dn = np.array([[self.X_moduu*abs(u), 0, 0, 0, 0, 0],
                   [0, self.Y_modvv*abs(v) + self.Y_modrv*abs(r) + self.Y_uvf*u, 0, 0, 0, self.Y_modvr*abs(v) + self.Y_modrr*abs(r)],
                   [0, 0, self.Z_modww*abs(w), 0, 0, 0],
                   [0, 0, 0, self.K_modpp*abs(p), 0, 0],
                   [0, 0, 0, 0, self.M_modqq*abs(q), 0],
                   [0, self.N_modvv*abs(v) + self.N_modrv*abs(r) + self.N_uvf*u, 0, 0, 0, self.N_modvr*abs(v) + self.N_modrr*abs(r)]])

    Tau = (self.Dl + Dn).dot(vel)

    #----------------------------------------------------------
    # Apply forces and moments in body frame for each hull

    # Hydrodynamic forces in body frame
    bge_object.applyForce(Tau[:3:],True)

    # Hydrodynamic moments in body frame
    bge_object.applyTorque(Tau[3::],True)

class WamVHydrodynamics(morse.core.actuator.Actuator):
    """Write here the general documentation of your actuator.
    It will appear in the generated online documentation.
    """
    _name = "Dynamics"
    _short_desc = "Real-time dynamics calculations for WAM-V"

    # define here the data fields required by your actuator
    # format is: field name, initial value, type, description
    add_property('_Cd', 0.6 ,'Drag_coeff', 'float', "Overall Drag coefficient for vehicle")    

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        # Density of seawater
        seaWaterRho = 1025
  
        # Get all bge objects in the scene
        objs = blenderapi.scene().objects

        # Get the parent robot
        robot = self.robot_parent.bge_object

        # Get the names of everything connected to this robot
        names = find_family(robot)
        compound_names = list(names)

        for n in names:
            children = objs[n].childrenRecursive
            compound_names.extend(c.name for c in children)

        # All bge objects comprising the robot
        compound_objs = [objs[n] for n in compound_names]

        # Get game objects belonging to this robot (Blender can rename them)
        self.port_hull = next(c for c in compound_objs if 'port_hull' in c.name)
        self.stbd_hull = next(c for c in compound_objs if 'stbd_hull' in c.name)
        self.port_prop = next(c for c in compound_objs if 'port_prop' in c.name)
        self.stbd_prop = next(c for c in compound_objs if 'stbd_prop' in c.name)

        # Linear drag derivatives
        self.X_u = -20 # Axial drag
        self.Y_v = -10 # Sway drag
        self.Y_r = -1  # Yaw drag
        self.Z_w = -100 # Heave drag
        self.N_r = -2  # Yaw drag moment
        self.N_v = -2  # Sway drag moment
        self.M_q = -2000 # Pitch drag moment

        # Linear damping matrix
        self.Dl = np.array([[self.X_u, 0,        0,        0, 0,        0],
                            [0,        self.Y_v, 0,        0, 0,        self.Y_r],
                            [0,        0,        self.Z_w, 0, 0,        0],
                            [0,        0,        0,        0, 0,        0],
                            [0,        0,        0,        0, self.M_q, 0],
                            [0,        self.N_v, 0,        0, 0,        self.N_r]])

        # Motor fin parameters
        ARe_fin = 2                               # Effective fin aspect ratio
        Ap_fin  = 0.2 * 0.1                       # Planform area of fin
        dCL_fin = 1/(1/(2*0.9*pi)+1/(pi*ARe_fin)) # Need a reference here...

        # Fin position needs to be relative to the CoG of each hull
        # The following fin displacement is the same for each hull
        fin_x = -self.port_prop.getDistanceTo(self.port_hull)
        fin_const = 0.5 * seaWaterRho * Ap_fin * dCL_fin

        # Motor fin lift coefficients
        self.Y_uudr =  fin_const
        self.Y_uvf  = -self.Y_uudr
        self.Y_urf  = -fin_const*fin_x
        self.N_uudr =  fin_const*fin_x
        self.N_uvf  = -self.N_uudr
        self.N_urf  = -fin_const*fin_x*fin_x

        # 2nd order damping coefficients
        self.X_moduu = -7    # Surge damping
        self.Y_modvv = -20   # Sway damping
        self.Z_modww = -1000  # Heave damping
        self.K_modpp =  0    # Roll damping
        self.M_modqq = -2000  # Pitch damping
        self.N_modrr = -20   # Yaw damping
        self.Y_modrr = -20
        self.Y_modrv =  0
        self.Y_modvr =  0
        self.N_modrv =  0
        self.N_modvv = -20
        self.N_modvr =  0

        # Get current object
        try:
            self.current = objs['fake.current']
        except:
            self.current = None
            logger.info('No current simulation found...')

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):

        # Main loop of the actuator.
        do_model(self,self.port_hull)
        do_model(self,self.stbd_hull)

        #logger.info('Executed default action')
