import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
from morse.core.actuator import Actuator
from morse.helpers.components import add_data, add_property

class Joystick(Actuator):
    """
    This actuator does not require a connection with external data. It
    simply responds to the joystick to generate movement instructions for
    the robot attached.
    """

    _name = "Joystick Actuator"
    _short_desc="A 'fake' actuator that allows to move a robot from the joystick."

    add_property('_type', 'Position', 'ControlType', 'string',
                 "Kind of control to move the parent robot, in ['Position', "
                 "'Velocity', 'Differential']")
    add_property('_speed', 1.0, 'Speed', 'float',
                 "Movement speed of the parent robot, in m/s")

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        Actuator.__init__(self, obj, parent)

        joysticks = blenderapi.joysticks()
        if joysticks.count(None) == len(joysticks):
            logger.error("No Joystick detected")
        else:
            logger.info("Found Joystick: " + repr(joysticks) )

        # Correct the speed considering the Blender clock
        if self._type == 'Position':
            self._speed = self._speed / self.frequency
        elif self._type == 'Differential':
            self._stopped = True
            # get track width for calculating wheel speeds from yaw rate
            parent = self.robot_parent
            self._trackWidth = parent._trackWidth
            self._radius = parent._wheelRadius

        """ Documentation says: ``bge.types.SCA_JoystickSensor.axisValues``
        Each specifying the value of an axis between -32767 and 32767 depending
        on how far the axis is pushed, 0 for nothing. The first 2 values are
        used by most joysticks and gamepads for directional control. 3rd and
        4th values are only on some joysticks and can be used for arbitary
        controls.
        """
        # speed_factor = - speed / max
        self._speed_factor = - self._speed / 32767.0

        logger.info('Component initialized')


    def default_action(self):
        """ Interpret joystick axis push and assign them to movement
            for the robot."""
        joystick_sensor = blenderapi.controller().sensors[0]
        # Reset movement variables
        vx, vy, vz = 0.0, 0.0, 0.0
        rx, ry, rz = 0.0, 0.0, 0.0

        rz = joystick_sensor.axisValues[0] * self._speed_factor
        vx = joystick_sensor.axisValues[1] * self._speed_factor

        # Give the movement instructions directly to the parent
        # The second parameter specifies a "local" movement
        if self._type == 'Position' or self._type == 'Velocity':
            self.apply_speed(self._type, [vx, vy, vz], [rx, ry, rz / 2.0])
        elif self._type == 'Differential':
            self.apply_vw_wheels(vx, -rz)

    # from v_omega_diff_drive
    def apply_vw_wheels(self, vx, vw):
        """ Apply (v, w) to the parent robot. """

        # calculate desired wheel speeds and set them
        if abs(vx) < 0.001 and abs(vw) < 0.001:
            # stop the wheel when velocity is below a given threshold
            for index in self.robot_parent._wheels.keys():
                self.robot_parent._wheel_joints[index].setParam(9, 0, 100.0)

            self._stopped = True
        else:
            # this is need to "wake up" the physic objects if they have
            # gone to sleep apply a tiny impulse straight down on the
            # object
            if self._stopped:
                self.robot_parent.bge_object.applyImpulse(
                   self.robot_parent.bge_object.position, (0.0, 0.1, -0.000001))

            # no longer stopped
            self._stopped = False

            # Another formula for computing left and right wheel speeds:
            # http://arri.uta.edu/acs/jmireles/Robotics/KinematicsMobileRobots.pdf
            v_ws_l = vx - (self._trackWidth / 2.0) * vw
            v_ws_r = vx + (self._trackWidth / 2.0) * vw

            # convert to angular speeds
            w_ws_l = v_ws_l / self._radius
            w_ws_r = v_ws_r / self._radius

            # set wheel speeds - front and rear wheels have the same speed
            # Left side wheels
            self.robot_parent._wheel_joints['FL'].setParam(9, w_ws_l, 100.0)
            if 'RL' in self.robot_parent._wheels:
                self.robot_parent._wheel_joints['RL'].setParam(9, w_ws_l, 100.0)
            # Right side wheels
            self.robot_parent._wheel_joints['FR'].setParam(9, w_ws_r, 100.0)
            if 'RR' in self.robot_parent._wheels:
                self.robot_parent._wheel_joints['RR'].setParam(9, w_ws_r, 100.0)

            logger.debug("New speeds set: left=%.4f, right=%.4f" %
                         (w_ws_l, w_ws_r))

