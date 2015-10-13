import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
from morse.core.actuator import Actuator
from morse.helpers.components import add_data, add_property

class Joystick(Actuator):
    """
    This actuator does not require a connection with external data. It
    simply responds to the joystick to generate movement instructions for
    the robot attached.


    .. example::

        from morse.builder import *

        # adds a default robot (the MORSE mascott!)
        robot = Morsy()

        # creates a new instance of the joystick actuator
        joystick = Joystick()
        robot.append(keyboard)

        env = Environment('empty')

        # Run this simulation: you can move the robot with your joystick.

    :noautoexample:
    """

    _name = "Joystick Actuator"
    _short_desc="A 'fake' actuator that allows to move a robot from the joystick."

    add_property('_type', 'Velocity', 'ControlType', 'string',
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

        self.zero_motion = True

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

        # Send a 'zero motion' only once in a row.
        if self.zero_motion and (vx,vy,vz,rx,ry,rz) == (0,0,0,0,0,0):
            return

        # Give the movement instructions directly to the parent
        # The second parameter specifies a "local" movement
        if self._type == 'Position' or self._type == 'Velocity':
            self.robot_parent.apply_speed(self._type, [vx, vy, vz], [rx, ry, rz / 2.0])
        elif self._type == 'Differential':
            self.robot_parent.apply_vw_wheels(vx, -rz)

        if (vx,vy,vz,rx,ry,rz) == (0,0,0,0,0,0):
            self.zero_motion = True
        else:
            self.zero_motion = False


