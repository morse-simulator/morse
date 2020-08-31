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
from morse.helpers.components import add_data, add_property
from math import degrees, radians, pi
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

# The following calculates approximate hydrodynamic
# forces for each WAM-V hull separately.
# The model follows Fossen's matrix treatment with the
# addition of some fin lift terms, without which the
# WAM-V appears to have virtually no directional stability.
# For now, calculations are in the Blender coordinate frame.

def do_model(self,bge_object):

    # Linear velocities in body frame
    linVel = bge_object.localLinearVelocity

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

        # Get game object handles
        # Only the boundaries are unparented
        # and hence have physics controllers
        self.port_thruster = objs['port_thruster_boundary']
        self.stbd_thruster = objs['stbd_thruster_boundary']
        self.port_hull     = objs['port_hull_boundary']
        self.stbd_hull     = objs['stbd_hull_boundary']
        self.port_prop     = objs['port_prop']
        self.stbd_prop     = objs['stbd_prop']

        # Linear drag derivatives
        self.X_u = -50 # Axial drag
        self.Y_v = -10 # Sway drag
        self.Y_r =  2  # Yaw drag
        self.N_r = -2  # Yaw drag moment
        self.N_v = -2  # Sway drag moment

        # Linear damping matrix
        self.Dl = np.array([[self.X_u, 0,        0, 0, 0, 0],
                            [0,        self.Y_v, 0, 0, 0, self.Y_r],
                            [0,        0,        0, 0, 0, 0],
                            [0,        0,        0, 0, 0, 0],
                            [0,        0,        0, 0, 0, 0],
                            [0,        self.N_v, 0, 0, 0, self.N_r]])

        # Motor fin parameters
        ARe_fin = 2                               # Effective fin aspect ratio
        Ap_fin  = 0.2*0.1                         # Planform area of fin
        dCL_fin = 1/(1/(2*0.9*pi)+1/(pi*ARe_fin)) # Need a reference here...

        # Fin position needs to be relative to the CoG of each hull
        # The following fin displacement is the same for each hull
        fin_x = -self.port_prop.getDistanceTo(self.port_hull)

        # Motor fin lift coefficients
        self.Y_uudr =  0.5*seaWaterRho*Ap_fin*dCL_fin
        self.Y_uvf  = -self.Y_uudr
        self.Y_urf  = -0.5*seaWaterRho*Ap_fin*dCL_fin*fin_x
        self.N_uudr =  0.5*seaWaterRho*Ap_fin*dCL_fin*fin_x
        self.N_uvf  = -self.N_uudr
        self.N_urf  = -0.5*seaWaterRho*Ap_fin*dCL_fin*fin_x*fin_x

        # 2nd order damping coefficients
        self.X_moduu = -7    # Surge damping
        self.Y_modvv = -20   # Sway damping
        self.Z_modww = -1000 # Heave damping
        self.K_modpp =  0    # Roll damping
        self.M_modqq = -2000 # Pitch damping
        self.N_modrr = -200  # Yaw damping
        self.Y_modrr = -20
        self.Y_modrv =  0
        self.Y_modvr =  0
        self.N_modrv =  0
        self.N_modvv = -20
        self.N_modvr =  0

        logger.info('Component initialized')

    def default_action(self):

        # Main loop of the actuator.
        do_model(self,self.port_hull)
        do_model(self,self.stbd_hull)

        #logger.info('Executed default action')
