import logging; logger = logging.getLogger("morse." + __name__)

from morse.core import blenderapi
from morse.core.mathutils import Vector, Matrix
from math import radians, degrees, sin, cos, fabs, copysign, sqrt
from morse.helpers.morse_math import normalise_angle
from morse.helpers.components import add_data, add_property

import morse.core.actuator
from morse.core.services import service, async_service, interruptible
from morse.core import status


class RotorcraftVelocity(morse.core.actuator.Actuator):
    """
    This controller will receive a velocity and yaw rate command
    and make the robot move by changing attitude.
    This controller is meant for rotorcrafts like quadrotors.
    """

    _name = "Rotorcraft Velocity motion controller"
    _short_desc = "Motion controller using force and torque to move a rotorcraft with a specified velocity."

    add_data('vx', 0.0, 'float', "desired x velocity in m/s")
    add_data('vy', 0.0, 'float', "desired y velocity in m/s")
    add_data('vz', 0.0, 'float', "desired z velocity in m/s")
    add_data('vyaw', 0.0, 'float', "desired yaw rate radians/s")
    add_data('tolerance', 0.2, 'float', "velocity tolerance in m/s")

    add_property('_h_pgain', 0.2, 'HorizontalPgain', 'float',
                 'proportional gain for the outer horizontal velocity [xy] loop')
    add_property('_h_dgain', 0.8, 'HorizontalDgain', 'float',
                 'derivative gain for the outer horizontal velocity[xy] loop')
    add_property('_v_pgain', 8, 'VerticalPgain', 'float',
                 'proportional gain for the vertical velocity loop')
    add_property('_v_dgain', 2, 'VerticalDgain', 'float',
                 'derivative gain for the vertical velocity loop')
    add_property('_yaw_pgain', 12.0, 'YawPgain', 'float',
                 'proportional gain for yaw control of the inner loop')
    add_property('_yaw_dgain', 8.0, 'YawDgain', 'float',
                 'derivative gain for yaw control of the inner loop')
    add_property('_rp_pgain', 9.7, 'RollPitchPgain', 'float',
                 'proportional gain for roll/pitch control of the inner loop')
    add_property('_rp_dgain', 2, 'RollPitchDgain', 'float',
                 'derivative gain for roll/pitch control of the inner loop')
    add_property('_max_bank_angle', radians(30), 'MaxBankAngle', 'float',
                 'limit the maximum roll/pitch angle of the robot. This \
                  effectively limits the horizontal acceleration of the robot')
    add_property('_max_velocity', 2, 'MaxVelocity', 'float',
                 'limit the maximum velocity of the robot.')

    def __init__(self, obj, parent=None):

        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        logger.setLevel(logging.INFO)

        robot_obj = self.robot_parent.bge_object

        # set desired velocity to zero
        self.local_data['vx'] = 0.0
        self.local_data['vy'] = 0.0
        self.local_data['vz'] = 0.0
        self.local_data['vyaw'] = 0.0


        # get the robot inertia (list [ix, iy, iz])
        robot_inertia = robot_obj.localInertia
        self.inertia = Vector(tuple(robot_inertia))
        logger.info("robot inertia: (%.3f %.3f %.3f)" % tuple(self.inertia))

        self.nominal_thrust = robot_obj.mass * 9.81
        logger.info("nominal thrust: %.3f", self.nominal_thrust)
        self._attitude_compensation_limit = cos(self._max_bank_angle) ** 2

        # current attitude setpoints in radians
        self.roll_setpoint = 0.0
        self.pitch_setpoint = 0.0
        self.yaw_setpoint = 0.0

        self.thrust = 0.0

        self._prev_vel_blender = Vector((0.0, 0.0, 0.0))
        self._prev_yaw = 0.0

        # previous attitude error
        self._prev_err = Vector((0.0, 0.0, 0.0))

        logger.info("Component initialized, runs at %.2f Hz ", self.frequency)


    @service
    def setvel(self, vx, vy, vz, vyaw, tolerance=0.1):
        """
        Set a new desired velocity and return.

        The robot will try to go achieve the desired velocity,
        but the service caller has no mean to know when the
        desired velocity is reached or if it failed.

        :param vx: x coordinate of the velocity (meter/sec)
        :param vy: y coordinate of the velocity (meter/sec)
        :param vz: z coordinate of the velocity (meter/sec)
        :param vyaw: yaw rate (radian/sec)
        :param tolerance: speed difference considered to decide
                          whether the desired velocity has been
                          reached (meter/sec)

        :return: True, except if the desired velocity is already reached.
                 In this case, returns False.
        """

        vel = sqrt(sum(v * v for v in self._lin_vel_sp))
        if vel - tolerance <= 0:
            logger.info("Robot already at target velocity ({})."
                    " I do not set a new velocity.".format(vel))
            return False

        self.local_data['vx'] = vx
        self.local_data['vy'] = vy
        self.local_data['vz'] = vz
        self.local_data['vyaw'] = vyaw
        self.local_data['tolerance'] = tolerance

        return True


    def default_action(self):
        """ Move the robot. """
        robot_obj = self.robot_parent.bge_object

        vel_body_sp = Vector((self.local_data['vx'],
                              self.local_data['vy'],
                              self.local_data['vz']))

        # TODO: limit max_velocity setpoint

        # current angles to horizontal plane in NED
        # (not quite, but approx good enough)
        roll = self.position_3d.roll
        pitch = self.position_3d.pitch
        yaw = self.position_3d.yaw

        # convert the commands in body frame to blender frame
        # in which frame do we want to command??

        #body2blender = Matrix.Rotation(radians(180), 3, 'X') * Matrix.Rotation(yaw, 3, 'Z')
        body2blender = Matrix.Rotation(yaw, 3, 'Z')
        vel_blender_sp = body2blender * vel_body_sp

        # current velocity of robot in Blender world frame
        vel_blender = robot_obj.worldLinearVelocity

        # errors in blender frame
        vel_error = vel_blender_sp - vel_blender
        # zero desired acceleration for now... no reference...
        accel_error = -(vel_blender - self._prev_vel_blender) * self.frequency


        logger.debug("velocity in blender frame: (% .3f % .3f % .3f)",
                     vel_blender[0], vel_blender[1], vel_blender[2])
        logger.debug("velocity setpoint:: (% .3f % .3f % .3f)",
                     vel_blender_sp[0], vel_blender_sp[1], vel_blender_sp[2])
        logger.debug("velocity error: (% .3f % .3f % .3f)",
                     vel_error[0], vel_error[1], vel_error[2])
        logger.debug("acceleration error: (% .3f % .3f % .3f)",
                     accel_error[0], accel_error[1], accel_error[2])

        cmd_blender_x = self._h_pgain * vel_error[0] + self._h_dgain * accel_error[0]
        cmd_blender_y = self._h_pgain * vel_error[1] + self._h_dgain * accel_error[1]

        self.roll_setpoint = sin(yaw) * cmd_blender_x - cos(yaw) * cmd_blender_y
        self.pitch_setpoint = cos(yaw) * cmd_blender_x + sin(yaw) * cmd_blender_y
        self.yaw_setpoint = self._prev_yaw - (self.local_data['vyaw'] / self.frequency)

        # saturate max roll/pitch angles
        if fabs(self.roll_setpoint) > self._max_bank_angle:
            self.roll_setpoint = copysign(self._max_bank_angle, self.roll_setpoint)
        if fabs(self.pitch_setpoint) > self._max_bank_angle:
            self.pitch_setpoint = copysign(self._max_bank_angle, self.pitch_setpoint)

        # wrap yaw setpoint
        self.yaw_setpoint = normalise_angle(self.yaw_setpoint)

        logger.debug("roll  current: % 2.3f   setpoint: % 2.3f",
                     degrees(roll), degrees(self.roll_setpoint))
        logger.debug("pitch current: % 2.3f   setpoint: % 2.3f",
                     degrees(pitch), degrees(self.pitch_setpoint))
        logger.debug("yaw   current: % 2.3f   setpoint: % 2.3f",
                     degrees(yaw), degrees(self.yaw_setpoint))

        # compute thrust
        # nominal command to keep altitude (feed forward)
        thrust_attitude_compensation = max(self._attitude_compensation_limit, cos(roll) * cos(pitch))
        thrust_ff = self.nominal_thrust / thrust_attitude_compensation
        # feedback to correct vertical speed
        thrust_fb = self._v_pgain * vel_error[2]# + self._v_dgain * accel_error[2]
        self.thrust = max(0, thrust_ff + thrust_fb)


        # Compute attitude errors
        #
        # e = att_sp - att = attitude error
        roll_err = normalise_angle(self.roll_setpoint - roll)
        pitch_err = normalise_angle(self.pitch_setpoint - pitch)
        yaw_err = normalise_angle(self.yaw_setpoint - yaw)

        err = Vector((roll_err, pitch_err, yaw_err))

        # derivative
        we = (err - self._prev_err) * self.frequency
        # we = Vector((0.0, 0.0, 0.0))
        # logger.debug("yaw rate error: %.3f", we[2])

        kp = Vector((self._rp_pgain, self._rp_pgain, self._yaw_pgain))
        kd = Vector((self._rp_dgain, self._rp_dgain, self._yaw_dgain))

        # torque = self.inertia * (kp * err + kd * we)
        t = []
        for i in range(3):
            t.append(self.inertia[i] * (kp[i] * err[i] + kd[i] * we[i]))
        # convert to blender frame and scale with thrust
        torque = Vector((t[0], t[1], t[2])) * self.thrust / self.nominal_thrust
        logger.debug("applied torques: (% .3f % .3f % .3f)", torque[0], torque[1], torque[2])

        force = Vector((0.0, 0.0, self.thrust))
        logger.debug("applied thrust force: %.3f", force[2])

        self._prev_err = err.copy()
        self._prev_vel_blender = vel_blender.copy()
        self._prev_yaw = yaw

        # directly apply local forces and torques to the blender object of the parent robot
        robot_obj.applyForce(force, True)
        robot_obj.applyTorque(torque, True)
