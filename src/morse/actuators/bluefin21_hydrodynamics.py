import logging; logger = logging.getLogger("morse." + __name__)
from math import degrees, radians, sin, cos, asin, acos, pi, atan2
from morse.helpers.components import add_data, add_property
from morse.helpers.morse_math import rotation_direction
from morse.core.mathutils import *
from morse.core import blenderapi
import morse.core.actuator
import numpy as np
import time

# Enforce an absolute limit on actuator value
def abs_limit(self,name,limit):

    if self.local_data[name] > limit:
        self.local_data[name] = limit
    elif self.local_data[name] < -limit:
        self.local_data[name] = -limit

# Calculate actuation increment, taking into
# account the current simulation frequency
def get_increment(val,desired,rate,tol,freq):

    if freq == 0:
        return 0 # Sorry - not ready yet
    
    speed = rate / freq

    return rotation_direction(val,desired,tol,speed)

class Hydrodynamics(morse.core.actuator.Actuator): 

    _name = "Hydrodynamics"
    _short_desc = "Hydrodynamics for Bluefin 21 AUV"

    # define here the data fields required by your actuator
    # format is: field name, initial value, type, description
    # The following are desired and instantaneous values respectively.
    add_data('desired_rudder',   0.0,'float','Desired desired_rudder rotation in degrees')
    add_data('desired_elevator', 0.0,'float','Desired desired_elevator rotation in degrees')
    add_data('desired_thrust',   0.0,'float','Desired propeller desired_thrust in percent')

    # Initialises some properties. They can be changed by Builder scripts
    add_property('_speed', 0.5, 'Slew_rate', 'float', 'Actuator rotation speed, in rad/s')
    add_property('_tolerance', radians(0.3), 'Slew_tol', 'float')

    def __init__(self, obj, parent=None): 

        logger.info("%s initialization" % obj.name)

        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        # Density of seawater
        seaWaterRho = 1025

        # Get the parent robot
        self.robot = parent.bge_object

        # Vehicle mass from Blender model
        self.mass = self.robot.mass

        # Get all the children
        children = self.robot.childrenRecursive

        # Handles to various Blender objects (They might have been renamed)
        self.tailcone = next(c for c in children if 'tailcone' in c.name)
        self.prop     = next(c for c in children if 'prop' in c.name)

        # Various vehicle-specific parameters
        A_f              = 1.14          # Frontal area in sq.m
        hullLength       = 3.5          # Streamlined body length (m)
        hullDiam         = 0.53          # Hull diameter (m)
        sFin             = 0.05          # Cross-sectional area of ring fin
        Cdc              = 1.1           # Cylinder drag coefficient (Hoerner)
        taper            = 2/3           # Fin taper ratio (for ring fin)
        Cdf              = 0.1+0.7*taper # Coefficient for ring-fin effect

        # Duct parameters
        self.ductDiam    = 0.415         # Tail duct diameter
        self.ductChord   = 0.12          # Tail duct chord
        self.dCLdAlpha   = 3.687         # Tail duct lift derivative
        self.CDDuctMin   = 0.01          # Min section duct drag coefficient
        self.CDDuctCF    = 0.81          # Cross-flow duct drag coefficient
        self.propDiam    = 0.375         # Propeller diameter

        self.maxRPM      = 500           # Maximum propeller RPM
        self.maxElevator = radians(13)   # Maximum elevator angle (deg)
        self.maxRudder   = radians(13)   # Maximum rudder angle (deg)

        # Instantaneous control surface positions
        self.rudder      = 0
        self.elevator    = 0
        self.thrust      = 0

        #-------------------------------------------------------------------
        # Parameters derived from vehicle geometry
 
        # Planform area of hull
        A_p = hullDiam*hullLength;

        # Lift/drag constants for cylindrical body
        RhoA_pon2 = 0.5*seaWaterRho*A_p;
        DonL = hullDiam/hullLength;

        # Axial drag coefficient for a blunt cylindrical body
        CD_body = 0.004*pi*A_p/A_f*(1+60*pow(DonL,3)+0.0025/DonL);

        # Tail duct lift/drag constant
        self.C_duct = 0.5*seaWaterRho*self.ductDiam*self.ductChord;

        # Position of tail duct in body frame
        self.ductPos = self.tailcone.localPosition

        # X-coordinate of duct
        xFin = self.ductPos.x

        # Body lift and moment coefficient slopes (for cylindrical body)
        dCL_pitch = 0.131
        dCL_yaw   = 0.131
        dCM_pitch = 0.123
        dCM_yaw   = 0.123

        # Axial and cross=flow drag coefficients
        # There are some numerical integrals here which could probably be calculated.
        # These parameters should be calculated from scratch for any torpedo-shaped vehicle
        self.X_uu = -0.5*seaWaterRho*A_f*CD_body
        self.Y_vv = -0.5*seaWaterRho*(Cdc*1.075+Cdf*sFin)
        self.Y_uv = -0.5*seaWaterRho*A_p*dCL_yaw
        self.Y_rr = -0.5*seaWaterRho*(Cdc*0.054-xFin*abs(xFin)*Cdf*sFin)
        self.Z_uw = -0.5*seaWaterRho*A_p*dCL_pitch
        self.Z_ww =  self.Y_vv
        self.Z_qq = -self.Y_rr
        self.M_uw =  0.5*seaWaterRho*A_p*hullLength*dCL_pitch
        self.M_ww =  0.5*seaWaterRho*(Cdc*0.060-xFin*Cdf*sFin)
        self.M_qq = -0.5*seaWaterRho*(Cdc*1.139+xFin**2*abs(xFin)*Cdf*sFin)
        self.N_uv = -0.5*seaWaterRho*A_p*hullLength*dCL_yaw
        self.N_vv = -self.M_ww
        self.N_rr =  self.M_qq
        self.K_pp = -100 # Rolling drag - too little and we wobble incessantly

        # Get every bge object in the scene
        self.bge_objs = blenderapi.scene().objects

        # Get current object
        try:
            self.current = self.bge_objs['fake.current']
        except:
            self.current = None
            logger.info('No current simulation found...')

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):

        # Main loop of the actuator.

        if self.current:
            # Current vector in world frame 
            current_world = self.current['vec']
        else:
            current_world = Vector([0,0,0])

        # Instantaneous values of control parameters
        self.rudder   = self.tailcone.localOrientation.to_euler().z
        self.elevator = self.tailcone.localOrientation.to_euler().y

        # Enforce rudder limits on desired values
        abs_limit(self,'desired_rudder',self.maxRudder)
        abs_limit(self,'desired_elevator',self.maxElevator)

        # Rudder increment for this cycle
        dtheta = get_increment(self.rudder,
                               self.local_data['desired_rudder'],
                               self._speed,self._tolerance,self.frequency)

        # Elevator increment for this cycle
        dalpha = get_increment(self.elevator,
                               self.local_data['desired_elevator'],
                               self._speed,self._tolerance,self.frequency)

        # Do tailcone increment
        if dtheta or dalpha:

            rot = Euler([0.0, dalpha, dtheta])
            self.tailcone.applyRotation(rot, True)

        # Thrust increment for this cycle
        if (self.local_data['desired_thrust']   > self.thrust and self.thrust < 100):
            self.thrust += 1
        elif (self.local_data['desired_thrust'] < self.thrust and self.thrust > -100):
            self.thrust -= 1

        # Propeller rotation increment for this cycle
        dphi = radians(.5)*self.thrust

        # Do propeller increment
        if dphi:
            
            rot = Euler([dphi,0.0,0.0])
            self.prop.applyRotation(rot,True)
        
        #----------------------------------------------------------
        # Body lift and drag

        # Orientation matrix for hull in world frame
        hull_mat_world = self.robot.worldOrientation

        # Matrix inverse for hull
        hull_inv = hull_mat_world.transposed()

        # Current vector relative to hull
        current_rel = hull_inv * current_world

        # Linear velocities in body (hull) frame
        linVel = self.robot.localLinearVelocity - current_rel

        # Angular velocities in body (hull) frame
        angVel = self.robot.localAngularVelocity

        # Body-frame linear velocity components
        u = linVel.x # Surge
        v = linVel.y # Sway
        w = linVel.z # Heave

        # Squared linear velocities in body frame
        # For use with drag terms:
        uabsu = u*abs(u)
        vabsv = v*abs(v)
        wabsw = w*abs(w)

        # Body-frame angular velocity components
        p = angVel.x # Roll
        q = angVel.y # Pitch
        r = angVel.z # Yaw
        
        # Squared angular velocities in body frame
        # For use with drag terms:
        pabsp = p*abs(p)
        qabsq = q*abs(q)
        rabsr = r*abs(r)

        # For use with non-drag terms:
        usq = u*u

        # Body lift and drag forces in body frame
        Fbb = [self.X_uu*uabsu,
               self.Y_uv*u*v + self.Y_vv*vabsv + self.Y_rr*rabsr,
               self.Z_uw*u*w + self.Z_ww*wabsw + self.Z_qq*qabsq]

        # Body lift and drag moments in body frame
        Mbb = [self.K_pp*pabsp,
               self.M_uw*u*w + self.M_ww*wabsw + self.M_qq*qabsq,
               self.N_uv*u*v + self.N_vv*vabsv + self.N_rr*rabsr]

        #----------------------------------------------------------
        # Propeller thrust and moments

        # Tailcone orientation matrix in hull frame                
        thrust_mat_local = self.tailcone.localOrientation

        # Matrix inverse for tailcone
        thrust_inv = thrust_mat_local.transposed()

        # Total water velocity in tailcone frame
        tcVel = thrust_inv * linVel

        rpm = self.thrust*self.maxRPM/100

        if (rpm == 0):
            Up = 0.0
            Tp = 0.0
            Mpp = [0,0,0]
        else:
            Up = tcVel.x * 0.9
            J  = Up/self.propDiam/abs(rpm)
            Kt = 0.035-0.3*J
            Tp = rpm*abs(rpm)*Kt*pow(self.propDiam,4.0)
            omega = rpm/60*2*pi

            # Approx prop torque
            Mpp = [Tp*Up/omega,0,0]

        # Duct to body frame transformation
        Rd2b = np.array(self.tailcone.localOrientation)

        # Propeller forces in duct frame
        Fpd = [Tp,0,0]

        # Propeller forces in body frame
        Fpb = Rd2b.dot(Fpd)

        # Propeller moments in body frame
        Mpb = np.cross(self.ductPos,Fpb)+Mpp
       
        #----------------------------------------------------------
        # Tail duct lift and drag

        # Duct velocity in body frame
        ductVel_b = np.array(linVel)+np.cross(angVel,self.ductPos)

        t1  = Rd2b.T.dot(ductVel_b)
        t2  = np.cross(t1,[1,0,0])
        t3  = np.cross(t1,t2)

        # Vector norms
        nt1 = np.linalg.norm(t1)+1e-30
        nt2 = np.linalg.norm(t2)+1e-30
        nt3 = np.linalg.norm(t3)+1e-30

        # Tail duct angle of attack
        Alpha = asin(nt2/nt1)

        # Duct to body-frame transformation
        Rr2d = np.column_stack([t1/nt1,t2/nt2,t3/nt3])

        # Tail duct lift coefficient
        CL_duct = self.dCLdAlpha*Alpha+self.CDDuctCF*Alpha*abs(Alpha)

        # Tail duct drag coefficient
        CD_duct = self.CDDuctMin+pow(CL_duct,2)*self.ductChord/(pi*self.ductDiam*0.9)

        # Tail duct lift force in duct frame
        L_duct = self.C_duct*usq*CL_duct

        # Tail duct drag force in duct frame
        D_duct = self.C_duct*uabsu*CD_duct

        # Tail duct and propeller forces in duct frame
        Fdr = [-D_duct,0,-L_duct]

        # Tail duct forces in body frame
        Fdb = Rd2b.dot(Rr2d).dot(Fdr)

        # Tail duct moments in body frame
        Mdb = np.cross(self.ductPos,Fdb)

        #----------------------------------------------------------
        # Apply forces and moments in body frame

        # Vehicle forces in body frame
        self.robot.applyForce(Fbb+Fdb+Fpb,True)

        # Vehicle moments in body frame
        self.robot.applyTorque(Mbb+Mdb+Mpb,True)

        #logger.info('Executed default action')
