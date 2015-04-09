import logging; logger = logging.getLogger("morse." + __name__)
import math
import morse.core.sensor
from morse.core import mathutils, blenderapi
from morse.helpers.components import add_data, add_property

"""
Important note:

    The 'logger.debug' instructions take some processor work, even if they are
    not displayed. For this reason, it is best to comment out these lines in
    the 'default_action' method.
"""

class IMU(morse.core.sensor.Sensor):
    """
    This sensor emulates an Inertial Measurement Unit (IMU), measuring
    the angular velocity and linear acceleration including acceleration
    due to gravity.

    If the robot has a physics controller, the velocities are directly
    read from it's properties ``localAngularVelocity`` and
    ``worldLinearVelocity``. Otherwise the velocities are calculated by
    simple differentiation. Linear acceleration is always computed by
    differentiation of the linear velocity. The measurements are given
    in the IMU coordinate system, so the location and rotation of the
    IMU with respect to the robot is taken into account.
    """

    _name = "Inertial measurement unit"

    add_data('angular_velocity', [0.0, 0.0, 0.0], "vec3<float>",
             'rates in IMU x, y, z axes (in radian . sec ^ -1)')
    add_data('linear_acceleration', [0.0, 0.0, 0.0], "vec3<float>",
             'acceleration in IMU x, y, z axes (in m . sec ^ -2)')
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
            self.robot_vel = self.robot_parent.bge_object.worldLinearVelocity
        else:
            # reference to sensor position
            self.pos = self.bge_object.worldPosition
            # previous position
            self.pp = self.pos.copy()
            # previous attitude euler angles as vector
            self.patt = mathutils.Vector(self.position_3d.euler)

        # previous linear velocity
        self.plv = mathutils.Vector((0.0, 0.0, 0.0))
        # previous angular velocity
        self.pav = mathutils.Vector((0.0, 0.0, 0.0))

        self.gravity = - blenderapi.gravity()

        # imu2body will transform a vector from imu frame to body frame
        self.imu2body = self.sensor_to_robot_position_3d()
        # rotate vector from body to imu frame
        self.rot_b2i = self.imu2body.rotation.conjugated()
        logger.debug("imu2body rotation RPY [% .3f % .3f % .3f]" % tuple(math.degrees(a) for a in self.imu2body.euler))
        logger.debug("imu2body translation [% .3f % .3f % .3f]" % tuple(self.imu2body.translation))

        if self.imu2body.translation.length > 0.01:
            self.compute_offset_acceleration = True
        else:
            self.compute_offset_acceleration = False

        # reference for rotating a vector from imu frame to world frame
        self.rot_i2w = self.bge_object.worldOrientation

        logger.info("IMU Component initialized, runs at %.2f Hz ", self.frequency)

    def sim_imu_simple(self):
        """
        Simulate angular velocity and linear acceleration measurements via simple differences.
        """
        # linear and angular velocities
        lin_vel = (self.pos - self.pp) * self.frequency
        att = mathutils.Vector(self.position_3d.euler)
        ang_vel = (att - self.patt) * self.frequency

        # linear acceleration in imu frame
        dv_imu = self.rot_i2w.transposed() * (lin_vel - self.plv) * self.frequency

        # measurement includes gravity and acceleration
        accel_meas = dv_imu + self.rot_i2w.transposed() * self.gravity

        # save current position and attitude for next step
        self.pp = self.pos.copy()
        self.patt = att
        # save velocity for next step
        self.plv = lin_vel
        self.pav = ang_vel

        return ang_vel, accel_meas

    def sim_imu_with_physics(self):
        """
        Simulate angular velocity and linear acceleration measurements using the physics of the robot.
        """

        # rotate the angular rates from the robot frame into the imu frame
        rates = self.rot_b2i * self.robot_w
        #logger.debug("rates in robot frame (% .4f, % .4f, % .4f)", self.robot_w[0], self.robot_w[1], self.robot_w[2])
        #logger.debug("rates in imu frame   (% .4f, % .4f, % .4f)", rates[0], rates[1], rates[2])

        # differentiate linear velocity in world (inertial) frame
        # and rotate to imu frame
        dv_imu = self.rot_i2w.transposed() * (self.robot_vel - self.plv) * self.frequency
        #logger.debug("velocity_dot in imu frame (% .4f, % .4f, % .4f)", dv_imu[0], dv_imu[1], dv_imu[2])

        # rotate acceleration due to gravity into imu frame
        g_imu = self.rot_i2w.transposed() * self.gravity

        # measurement includes gravity and acceleration
        accel_meas = dv_imu + g_imu

        if self.compute_offset_acceleration:
            # acceleration due to rotation (centripetal)
            # is zero if imu is mounted in robot center (assumed axis of rotation)
            a_centripetal = self.rot_b2i * rates.cross(rates.cross(self.imu2body.translation))
            #logger.debug("centripetal acceleration (% .4f, % .4f, % .4f)", a_rot[0], a_rot[1], a_rot[2])

            # linear acceleration due to angular acceleration
            a_alpha = self.rot_b2i * (self.robot_w - self.pav).cross(self.imu2body.translation) * self.frequency

            # final measurement includes acceleration due to rotation center not in IMU
            accel_meas += a_centripetal + a_alpha

        # save velocity for next step
        self.plv = self.robot_vel.copy()
        self.pav = self.robot_w.copy()

        return rates, accel_meas

    def default_action(self):
        """
        Get the speed and acceleration of the robot and transform it into the imu frame
        """
        if self._type == 'Velocity':
            (rates, accel) = self.sim_imu_with_physics()
        else:
            (rates, accel) = self.sim_imu_simple()

        # Store the important data
        self.local_data['angular_velocity'] = rates
        self.local_data['linear_acceleration'] = accel
