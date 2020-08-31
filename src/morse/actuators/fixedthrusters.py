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
from morse.core import status
from morse.helpers.components import add_data, add_property
from morse.core import blenderapi
from morse.core.mathutils import *
from math import radians

__author__     = "David Battle"
__copyright__  = "Copyright 2017, Mission Systems Pty Ltd"
__license__    = "BSD"
__version__    = "1.0.2"
__maintainer__ = "David Battle"
__email__      = "david.battle@missionsystems.com.au"
__status__     = "Production"

# Check for submergence
def submerged(bge_object):

    # Vector pointing up
    up = Vector([0,0,100])

    target = bge_object.localPosition + up
    _,point,_ = bge_object.rayCast(target, None, 100, "castable", False, True)

    if point:
        return True

    return False

# Calculate prop force in Newtons
def prop_force(self,thrust,u):

    # RPM from thrust value
    rpm = thrust * self.max_RPM

    if (rpm == 0):
        return 0
    else:
        Up = u * 0.9
        J  = Up / self.prop_diam / abs(rpm)
        Kt = 0.035 - 0.3 * J

    return rpm * abs(rpm) * Kt * pow(self.prop_diam,4.0)

class Fixedthrusters(morse.core.actuator.Actuator):
    """Write here the general documentation of your actuator.
    It will appear in the generated online documentation.
    """
    _name = "Fixedthruster"
    _short_desc = "Simple model for fixed marine thrusters"

    # define here the data fields required by your actuator
    # format is: field name, initial value, type, description
    add_data('desired_thrust', 0.0, 'float', 'Desired overall thrust')
    add_data('desired_rudder', 0.0, 'float', 'Desired rudder value')

    # Initialises a couple of properties. They can be changed by Builder scripts
    add_property('max_RPM',     1200, 'Max_rpm'  ,   'float', 'Max motor RPM')
    add_property('prop_diam',    0.3, 'Prop_diam',   'float', 'Propeller diameter')
    add_property('thrust_scale', 100, 'Thrust_scale','float', 'Scaling factor to [-1:1] thrust values')
    add_property('thrust_rate', 0.25, 'Thrust_rate', 'float', 'Normalised slew rate for thruster')
    add_property('rudder_scale', 100, 'Rudder_scale','float', 'Scaling factor to [-1:1] rudder values')

    def __init__(self, obj, parent=None):
        logger.info("%s initialization" % obj.name)

        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        # Get every bge object in the scene
        objs = blenderapi.scene().objects

        # Get game object handles
        # Only the boundaries are unparented
        # and hence have physics controllers
        self.port_thruster = objs['port_thruster_boundary']
        self.stbd_thruster = objs['stbd_thruster_boundary']
        self.port_prop     = objs['port_prop']
        self.stbd_prop     = objs['stbd_prop']

        # Instantaneous thrust
        self.port_thrust = 0
        self.stbd_thrust = 0

        logger.info('Component initialized')

    def default_action(self):

        # Normalise commanded values
        rudder = self.local_data['desired_rudder'] / self.rudder_scale
        thrust = self.local_data['desired_thrust'] / self.thrust_scale

        if rudder > 1:
            rudder = 1
        elif rudder < -1:
            rudder = -1

        if thrust > 1:
        	thrust = 1
        elif thrust < -1:
            thrust = -1

        # Differential thrust for directional control
        desired_port_thrust = thrust + 2 * rudder
        desired_stbd_thrust = thrust - 2 * rudder

        # Port thrust increment for this cycle
        if (desired_port_thrust > self.port_thrust and self.port_thrust < 1):
            self.port_thrust += self.thrust_rate / self.frequency
        elif (desired_port_thrust < self.port_thrust and self.port_thrust > -1):
            self.port_thrust -= self.thrust_rate / self.frequency

        # Stbd thrust increment for this cycle
        if (desired_stbd_thrust > self.stbd_thrust and self.stbd_thrust < 1):
            self.stbd_thrust += self.thrust_rate / self.frequency
        elif (desired_stbd_thrust < self.stbd_thrust and self.stbd_thrust > -1):
            self.stbd_thrust -= self.thrust_rate / self.frequency

        #----------------------------------------------------------
        # Propeller rotation increments:
        # Mostly eye candy, but useful for
        # seeing what the thrusters are doing
        dphi = radians(40)*self.port_thrust

        if dphi:

            rot = Euler([dphi,0.0,0.0])
            self.port_prop.applyRotation(rot,True)

        dphi = radians(40)*self.stbd_thrust

        if dphi:

            rot = Euler([dphi,0.0,0.0])
            self.stbd_prop.applyRotation(rot,True)

        #----------------------------------------------------------
        # Apply thrust to props (if submerged):
        # (Water level may not be zero)

        if submerged(self.port_prop):
            u_port = self.port_thruster.localLinearVelocity.x
            port_force = prop_force(self,self.port_thrust,u_port)
            port_impulse = [port_force / self.frequency,0,0]
            self.port_thruster.applyImpulse(self.port_prop.localPosition,port_impulse,True)

        if submerged(self.stbd_prop):
            u_stbd = self.stbd_thruster.localLinearVelocity.x
            stbd_force = prop_force(self,self.stbd_thrust,u_stbd)
            stbd_impulse = [stbd_force / self.frequency,0,0]
            self.stbd_thruster.applyImpulse(self.stbd_prop.localPosition,stbd_impulse,True)
