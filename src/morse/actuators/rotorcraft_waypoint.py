import logging; logger = logging.getLogger("morse." + __name__)

from morse.core import blenderapi
from morse.core.mathutils import Vector, Matrix
from math import radians, degrees, sin, cos, fabs, copysign
from morse.helpers.morse_math import normalise_angle
from morse.helpers.components import add_data, add_property

import morse.core.actuator
from morse.core.services import service, async_service, interruptible
from morse.core import status


class RotorcraftWaypoint(morse.core.actuator.Actuator):
    """
    This controller will receive a 3D destination point and heading
    and make the robot move to that location by changing attitude.
    This controller is meant for rotorcrafts like quadrotors.
    """

    _name = "Rotorcraft Waypoint motion controller"
    _short_desc = "Motion controller using force and torque to move a rotorcraft to a waypoint."

    add_data('x', 0.0, 'float', "waypoint x coordinate in meters")
    add_data('y', 0.0, 'float', "waypoint y coordinate in meters")
    add_data('z', 0.0, 'float', "waypoint z coordinate in meters")
    add_data('yaw', 0.0, 'float', "desired heading angle in radians")
    add_data('tolerance', 0.2, 'float', "waypoint tolerance in meters")

    add_property('_h_pgain', radians(6), 'HorizontalPgain', 'float',
                 'proportional gain for the outer horizontal position [xy] loop')
    add_property('_h_dgain', radians(8), 'HorizontalDgain', 'float',
                 'derivative gain for the outer horizontal position [xy] loop')
    add_property('_v_pgain', 8, 'VerticalPgain', 'float',
                 'proportional gain for the altitude loop')
    add_property('_v_dgain', 8, 'VerticalDgain', 'float',
                 'derivative gain for the altitude loop')
    add_property('_yaw_pgain', 12.0, 'YawPgain', 'float',
                 'proportional gain for yaw control of the inner loop')
    add_property('_yaw_dgain', 6.0, 'YawDgain', 'float',
                 'derivative gain for yaw control of the inner loop')
    add_property('_rp_pgain', 9.7, 'RollPitchPgain', 'float',
                 'proportional gain for roll/pitch control of the inner loop')
    add_property('_rp_dgain', 2, 'RollPitchDgain', 'float',
                 'derivative gain for roll/pitch control of the inner loop')
    add_property('_max_bank_angle', radians(30), 'MaxBankAngle', 'float',
                 'limit the maximum roll/pitch angle of the robot. This \
                  effectively limits the horizontal acceleration of the robot')
    add_property('_target', 'wp_target', 'Target', 'string',
                 'name of a blender object in the scene. When specified, \
                  this object will be placed at the coordinates given to \
                  the actuator, to indicate the expected destination of  \
                  the robot. Make sure that this object has NO_COLLISION')

    def __init__(self, obj, parent=None):

        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        logger.setLevel(logging.INFO)

        self._destination = self.robot_parent.bge_object.worldPosition
        self._wp_object = None

        # set desired position to current position
        self.local_data['x'] = self._destination[0]
        self.local_data['y'] = self._destination[1]
        self.local_data['z'] = self._destination[2]
        self.local_data['yaw'] = self.robot_parent.position_3d.yaw

        logger.info("inital wp: (%.3f %.3f %.3f)", self._destination[0], self._destination[0], self._destination[0])
        self._pos_initalized = False

        # Make new reference to the robot velocities (mathutils.Vector)
        self.robot_w = self.robot_parent.bge_object.localAngularVelocity

        # get the robot inertia (list [ix, iy, iz])
        robot_inertia = self.robot_parent.bge_object.localInertia
        self.inertia = Vector(tuple(robot_inertia))
        logger.info("robot inertia: (%.3f %.3f %.3f)" % tuple(self.inertia))

        self.nominal_thrust = self.robot_parent.bge_object.mass * 9.81
        logger.info("nominal thrust: %.3f", self.nominal_thrust)
        self._attitude_compensation_limit = cos(self._max_bank_angle) ** 2

        # current attitude setpoints in radians
        self.roll_setpoint = 0.0
        self.pitch_setpoint = 0.0
        self.yaw_setpoint = 0.0

        self.thrust = 0.0

        #previous attitude error
        self.prev_err = Vector((0.0, 0.0, 0.0))

        # Identify an object as the target of the motion
        try:
            wp_name = self._target
            if wp_name != '':
                scene = blenderapi.scene()
                self._wp_object = scene.objects[wp_name]
                logger.info("Using object '%s' to indicate motion target", wp_name)
        except KeyError as detail:
            self._wp_object = None
            logger.info("Not using a target object")

        logger.info("Component initialized, runs at %.2f Hz ", self.frequency)


    @service
    def setdest(self, x, y, z, yaw, tolerance=0.2):
        """
        Set a new waypoint and returns.

        The robot will try to go to this position, but the service 
        caller has no mean to know when the destination is reached
        or if it failed.

        In most cases, the asynchronous service 'goto' should be
        preferred.

        :param x: x coordinate of the waypoint (meter)
        :param y: y coordinate of the waypoint (meter)
        :param z: z coordinate of the waypoint (meter)
        :param yaw: orientation of the waypoint (radian)
        :param tolerance: distance considered to decide that the
                          waypoint has been reached (meter)

        :return: True (if the robot is already moving, the previous
                 target is replaced by the new one) except if the
                 destination is already reached. In this case, returns
                 False.
        """

        distance, gv, lv = self.robot_parent.bge_object.getVectTo([x, y, z])
        if distance - tolerance <= 0:
            logger.info("Robot already at destination (distance = {})."
                    " I do not set a new destination.".format(distance))
            return False

        self.local_data['x'] = x
        self.local_data['y'] = y
        self.local_data['z'] = z
        self.local_data['yaw'] = yaw
        self.local_data['tolerance'] = tolerance

        return True


    @interruptible
    @async_service
    def goto(self, x, y, z, yaw, tolerance=0.2):
        """ 
        Go to a new destination.

        The service returns when the destination is reached within
        the provided tolerance bounds.

        :param x: x coordinate of the waypoint (meter)
        :param y: y coordinate of the waypoint (meter)
        :param z: z coordinate of the waypoint (meter)
        :param yaw: orientation of the waypoint (radian)
        :param tolerance: distance considered to decide that the
                          waypoint has been reached (meter)
        """
        self.local_data['x'] = x
        self.local_data['y'] = y
        self.local_data['z'] = z
        self.local_data['yaw'] = yaw
        self.local_data['tolerance'] = tolerance


    @service
    def get_status(self):
        """ 
        Return the current status (Transit or Arrived)
        """
        return self.robot_parent.move_status


    def default_action(self):
        """ Move the object towards the destination. """
        robot = self.robot_parent

        if self._pos_initalized:
            self._destination = Vector((self.local_data['x'], self.local_data['y'], self.local_data['z']))
        else:
            self._destination = self.robot_parent.bge_object.worldPosition
            self.local_data['x'] = self._destination[0]
            self.local_data['y'] = self._destination[1]
            self.local_data['z'] = self._destination[2]
            self.local_data['yaw'] = self.robot_parent.position_3d.yaw
            self._pos_initalized = True

        #logger.debug("Robot %s move status: '%s'", robot.bge_object.name, robot.move_status)
        # Place the target marker where the robot should go
        if self._wp_object:
            self._wp_object.worldPosition = self._destination
            self._wp_object.worldOrientation = Matrix.Rotation(self.local_data['yaw'], 3, 'Z')

        # current angles to horizontal plane (not quite, but approx good enough)
        roll = self.position_3d.roll
        pitch = self.position_3d.pitch
        yaw = self.position_3d.yaw

        # current position and velocity of robot 
        pos_blender = robot.bge_object.worldPosition
        vel_blender = robot.bge_object.worldLinearVelocity

        pos_error = self._destination - pos_blender
        # zero velocity setpoint for now
        vel_error = -vel_blender

        logger.debug("pos current: (% .3f % .3f % .3f) setpoint: (% .3f % .3f % .3f)", \
                     pos_blender[0], pos_blender[1], pos_blender[2], \
                     self._destination[0], self._destination[1], self._destination[2])
        logger.debug("velocity: (% .3f % .3f % .3f)", vel_blender[0], vel_blender[1], vel_blender[2])

        # simple PD controller on horizontal position
        command_world_x = self._h_pgain * pos_error[0] + self._h_dgain * vel_error[0]
        command_world_y = self._h_pgain * pos_error[1] + self._h_dgain * vel_error[1]

        # setpoints in body frame
        self.roll_setpoint = sin(yaw) * command_world_x - cos(yaw) * command_world_y
        self.pitch_setpoint = cos(yaw) * command_world_x + sin(yaw) * command_world_y
        self.yaw_setpoint = self.local_data['yaw']

        # saturate max roll/pitch angles
        if fabs(self.roll_setpoint) > self._max_bank_angle:
            self.roll_setpoint = copysign(self._max_bank_angle, self.roll_setpoint)
        if fabs(self.pitch_setpoint) > self._max_bank_angle:
            self.pitch_setpoint = copysign(self._max_bank_angle, self.pitch_setpoint)

        # wrap yaw setpoint
        self.yaw_setpoint = normalise_angle(self.yaw_setpoint)

        logger.debug("roll  current: % 2.3f   setpoint: % 2.3f", \
                     degrees(roll), degrees(self.roll_setpoint))
        logger.debug("pitch current: % 2.3f   setpoint: % 2.3f", \
                     degrees(pitch), degrees(self.pitch_setpoint))
        logger.debug("yaw   current: % 2.3f   setpoint: % 2.3f", \
                     degrees(yaw), degrees(self.yaw_setpoint))


        # compute thrust
        # nominal command to keep altitude (feed forward)
        thrust_attitude_compensation = max(self._attitude_compensation_limit, cos(roll) * cos(pitch))
        thrust_ff = self.nominal_thrust / thrust_attitude_compensation
        # feedback to correct altitude
        thrust_fb = self._v_pgain * pos_error[2] + self._v_dgain * vel_error[2]
        self.thrust = max(0, thrust_ff + thrust_fb)


        # Compute attitude errors
        #
        # e = att_sp - att = attitude error
        roll_err = normalise_angle(self.roll_setpoint - roll)
        pitch_err = normalise_angle(self.pitch_setpoint - pitch)
        yaw_err = normalise_angle(self.yaw_setpoint - yaw)

        err = Vector((roll_err, pitch_err, yaw_err))

        # derivative
        we = (err - self.prev_err) * self.frequency
        #we = mathutils.Vector((0.0, 0.0, 0.0))
        #logger.debug("yaw rate error: %.3f", we[2])

        kp = Vector((self._rp_pgain, self._rp_pgain, self._yaw_pgain))
        kd = Vector((self._rp_dgain, self._rp_pgain, self._yaw_dgain))

        #torque = self.inertia * (kp * err + kd * we)
        t = []
        for i in range(3):
            t.append(self.inertia[i] * (kp[i] * err[i] + kd[i] * we[i]))
        # scale with thrust
        torque = Vector((t[0], t[1], t[2])) * self.thrust / self.nominal_thrust
        logger.debug("applied torques: (% .3f % .3f % .3f)", torque[0], torque[1], torque[2])

        force = Vector((0.0, 0.0, self.thrust))
        logger.debug("applied thrust force: %.3f", force[2])

        self.prev_err = err.copy()

        # directly apply local forces and torques to the blender object of the parent robot
        robot.bge_object.applyForce(force, True)
        robot.bge_object.applyTorque(torque, True)


        # Vectors returned are already normalized
        wp_distance, global_vector, local_vector = self.bge_object.getVectTo(self._destination)

        #logger.debug("GOT DISTANCE: xyz: %.4f", wp_distance)

        # If the target has been reached, change the status
        if wp_distance - self.local_data['tolerance'] <= 0:
            robot.move_status = "Arrived"

            #Do we have a running request? if yes, notify the completion
            self.completed(status.SUCCESS, robot.move_status)

            #logger.debug("TARGET REACHED")
            #logger.debug("Robot %s move status: '%s'", robot.bge_object.name, robot.move_status)

        else:
            robot.move_status = "Transit"
