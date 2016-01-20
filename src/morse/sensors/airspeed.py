import logging
logger = logging.getLogger("morse." + __name__)
from morse.core.sensor import Sensor
from morse.helpers.components import add_data, add_property
from morse.helpers.velocity import linear_velocities
from copy import copy

from math import pow

AIR_DENSITY = 1.225 # kg / m³

class Airspeed(Sensor):
    """
    Sensor to compute Airspeed

    - https://en.wikipedia.org/wiki/Airspeed

    The current implementation is only correct for speed < 1Mach
    """
    _name = "Airspeed"
    _short_desc = "Mesure airspeed"

    add_data('diff_pressure', 0.0, "float", 'Diff pressure in Pa')
    add_property('_type', 'Automatic', 'ComputationMode', 'string',
                 "Kind of computation, can be one of ['Velocity', 'Position']. "
                 "Only robot with dynamic and Velocity control can choose Velocity "
                 "computation. Default choice is Velocity for robot with physics, "
                 "and Position for others")

    def __init__(self, obj, parent=None):
        """ Constructor method.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        Sensor.__init__(self, obj, parent)

        has_physics = bool(self.robot_parent.bge_object.getPhysicsId())
        if self._type == 'Automatic':
            if has_physics: 
                self._type = 'Velocity'
            else:
                self._type = 'Position'

        if self._type == 'Velocity' and not has_physics:
            logger.error("Invalid configuration : Velocity computation without "
                        "physics")
            return

        if self._type == 'Velocity':
            self.robot_vel_body = self.robot_parent.bge_object.localLinearVelocity
            self.airspeed2body = self.sensor_to_robot_position_3d()
            # rotate vector from body to airspeed frame
            self.rot_b2a = self.airspeed2body.rotation.conjugated()
        else:
            self.pp = copy(self.position_3d)

        logger.info('Component initialized, runs at %.2f Hz', self.frequency)

    def default_action(self):
        if self._type == 'Velocity':
            vel = self.rot_b2a * self.robot_vel_body
        else:
            vel = linear_velocities(self.pp, self.position_3d, 1 / self.frequency)
            self.pp = copy(self.position_3d)

        v_x = vel[0]
        self.local_data['diff_pressure'] = 0.5 * AIR_DENSITY * v_x * v_x


