import logging; logger = logging.getLogger("morse." + __name__)

import morse.core.actuator
from math import degrees
from mathutils import Vector
from morse.helpers.morse_math import normalise_angle

class RotorcraftAttitudeActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller for RollPitchYawThrust control input

    This class will read desired attitude and thrust as input
    from an external middleware.
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        #logger.setLevel(logging.DEBUG)

        # roll input in radians
        self.local_data['roll'] = 0.0
        # pitch input in radians
        self.local_data['pitch'] = 0.0
        # yaw rate in rad/s
        self.local_data['yaw'] = 0.0
        # Collective: 0 .. 1 (= 0 .. 100%)
        self.local_data['thrust'] = 0.0

        self.add_property('_rp_pgain', 100.0, 'RollPitchPgain')
        self.add_property('_rp_dgain', 20.0, 'RollPitchDgain')
        self.add_property('_yaw_pgain', 16.0, 'YawPgain')
        self.add_property('_yaw_dgain', 4.0, 'YawDgain')
        self.add_property('_thrust_factor', 40.0, 'ThrustFactor')

        # Make new reference to the robot velocities (mathutils.Vector)
        self.robot_w = self.robot_parent.blender_obj.localAngularVelocity

        # get the robot inertia (list [ix, iy, iz])
        robot_inertia = self.robot_parent.blender_obj.localInertia
        self.inertia = Vector(tuple(robot_inertia))
        logger.debug("robot inertia: (%.3f %.3f %.3f)" % tuple(self.inertia))

        # yaw setpoint in radians is just integrated from yaw rate input
        self.yaw_setpoint = 0.0

        self.prev_err = Vector((0.0, 0.0, 0.0))

        logger.info("Component initialized, runs at %.2f Hz ", self.frequency)


    def default_action(self):
        """ Run attitude controller and apply resulting force and torque to the parent robot. """
        # Get the the parent robot
        robot = self.robot_parent

        if self.local_data['thrust'] > 0:
            # yaw_rate and yaw_setpoint in NED
            self.yaw_setpoint += self.local_data['yaw'] / self.frequency
            # wrap angle
            self.yaw_setpoint = normalise_angle(self.yaw_setpoint)

            logger.debug("yaw setpoint: %.3f", degrees(self.yaw_setpoint))
            logger.debug("yaw current: %.3f   setpoint: %.3f", -degrees(self.position_3d.yaw), degrees(self.yaw_setpoint))

            # Compute errors
            #
            # e = att_sp - att = attitude error
            # current angles to horizontal plane in NED
            roll = self.position_3d.roll
            pitch = -self.position_3d.pitch
            yaw = -self.position_3d.yaw
            roll_err = self.local_data['roll'] - roll
            pitch_err = self.local_data['pitch'] - pitch
            # wrapped yaw error
            yaw_err = normalise_angle(self.yaw_setpoint - yaw)

            err = Vector((roll_err, pitch_err, yaw_err))
            logger.debug("attitude error: (% .3f % .3f % .3f)", degrees(err[0]), degrees(err[1]), degrees(err[2]))

            # derivative
            we = (err - self.prev_err) * self.frequency
            logger.debug("yaw rate error: %.3f", we[2])

            kp = Vector((self._rp_pgain, self._rp_pgain, self._yaw_pgain))
            kd = Vector((self._rp_dgain, self._rp_pgain, self._yaw_dgain))

            # torque = self.inertia * (kp * err + kd * we)
            t = []
            for i in range(3):
                t.append(self.inertia[i] * (kp[i] * err[i] + kd[i] * we[i]))
            # convert to blender frame and scale with thrust
            torque = Vector((t[0], -t[1], -t[2])) * self.local_data['thrust']
            logger.debug("applied torques: (% .3f % .3f % .3f)", torque[0], torque[1], torque[2])

            force = Vector((0.0, 0.0, self.local_data['thrust'] * self._thrust_factor))
            logger.debug("applied thrust force: %.3f", force[2])

            self.prev_err = err.copy()
        else:
            force = Vector((0.0, 0.0, 0.0))
            torque = Vector((0.0, 0.0, 0.0))

        # directly apply local forces and torques to the blender object of the parent robot
        robot.blender_obj.applyForce(force, True)
        robot.blender_obj.applyTorque(torque, True)
