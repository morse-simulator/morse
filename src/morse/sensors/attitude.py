import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.sensor
from morse.core import mathutils, blenderapi
from morse.helpers.components import add_data, add_property
from morse.helpers.coordinates import CoordinateConverter
from morse.helpers.velocity import angular_velocities
from morse.helpers.morse_math import normalise_angle
from copy import copy

class Attitude(morse.core.sensor.Sensor):
    """
    This sensor is an high-level sensor, returning the attitude of the
    sensor (i.e. angles and  angular velocities). It can be seen as the
    integration of an IMU, or a couple gyroscope/gyrometer.

    If the robot has a physics controller, the velocities are directly
    read from its property ``localAngularVelocity``. Otherwise the
    velocities are calculated by simple differentiation.  The
    measurements are given in the sensor coordinate system.
    """

    _name = "Attitude sensor"

    add_data('rotation', [0.0, 0.0, 0.0], "Euler angles (XYZ)", 
             'rotation of the sensor, in radian')
    add_data('angular_velocity', [0.0, 0.0, 0.0], "vec3<float>",
             'rates in the sensors axis x, y, z axes (in radian . sec ^ -1)')
    add_property('_use_angle_against_north', False, 'UseAngleAgainstNorth', 'bool',
                 "If set to true, return the absolute yaw against North. The whole "
                 "geodetic coordinates (longitude, latitude, altitude, angle_against_north)"
                 " must be configured. Otherwise, return the yaw against the Blender "
                 "coordinates")
    add_property('_type', 'Automatic', 'ComputationMode', 'string',
                 "Kind of computation, can be one of ['Velocity', 'Position']. "
                 "Only robot with dynamic and Velocity control can choose Velocity "
                 "computation. Default choice is Velocity for robot with physics, "
                 "and Position for others")

    def __init__(self, obj, parent=None):
        """ Constructor method.

        Receives the reference to the Blender object.
        The second parameter should be the name of the object's parent.
        """
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.sensor.Sensor.__init__(self, obj, parent)

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
            # make new references to the robot velocities and use those.
            self.robot_w = self.robot_parent.bge_object.localAngularVelocity
        else:
            # previous attitude euler angles as vector
            self.pp = copy(self.position_3d)

        if self._use_angle_against_north:
            self._coord_converter = CoordinateConverter.instance()

        # imu2body will transform a vector from imu frame to body frame
        self.imu2body = self.sensor_to_robot_position_3d()
        # rotate vector from body to imu frame
        self.rot_b2i = self.imu2body.rotation.conjugated()

        logger.info("Attitude Component initialized, runs at %.2f Hz ", self.frequency)

    def sim_attitude_simple(self):
        """
        Simulate angular velocity measurements via simple differences.
        """
        # linear and angular velocities
        rates = angular_velocities(self.pp, self.position_3d, 1 / self.frequency)
        self.pp = copy(self.position_3d)

        return rates

    def sim_attitude_with_physics(self):
        """
        Simulate angular velocity using the physics of the robot.
        """
        return  self.rot_b2i * self.robot_w

    def default_action(self):
        if self._type == 'Velocity':
            rates = self.sim_attitude_with_physics()
        else:
            rates = self.sim_attitude_simple()

        # Store the important data
        self.local_data['rotation'] = self.position_3d.euler
        if self._use_angle_against_north:
            self.local_data['rotation'][2] = \
            normalise_angle(
                - self._coord_converter.angle_against_geographic_north(self.position_3d.euler))
        self.local_data['angular_velocity'] = rates
