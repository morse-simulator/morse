import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
from morse.core.actuator import Actuator
from morse.helpers.components import add_data, add_property

class Keyboard(Actuator):
    """
    This actuator does not require a connection with external data. It
    simply responds to the keyboard arrows to generate movement
    instructions for the robot attached.

    When parented to a robot, the user can press the arrow keys to modify the
    linear and angular velocities (V, W) of the robot.

    :kbd:`Up` forward
    :kbd:`Down` backwards
    :kbd:`Left` turn/yaw left
    :kbd:`Right` turn/yaw right
    :kbd:`I` pitch forward
    :kbd:`K` pitch backward
    :kbd:`L` roll left
    :kbd:`M` roll right
    :kbd:`U` strafe left
    :kbd:`O` strafe right
    :kbd:`T` move up
    :kbd:`G` move down

    .. example::

        from morse.builder import *

        # adds a default robot (the MORSE mascott!)
        robot = Morsy()

        # creates a new instance of the actuator
        keyboard = Keyboard()
        robot.append(keyboard)

        env = Environment('empty')

        # Run this simulation: you can move the robot with the arrow keys.

    :noautoexample:
    """

    _name = "Keyboard Actuator"
    _short_desc="A 'fake' actuator that allows to move a robot from the keyboard."

    add_property('_type', 'Position', 'ControlType', 'string',
                 "Kind of control to move the parent robot, in ['Position', "
                 "'Velocity', 'Differential']")
    add_property('_speed', 1.0, 'Speed', 'float',
                 "Movement speed of the parent robot, in m/s")

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        Actuator.__init__(self, obj, parent)

        # Correct the speed considering the Blender clock
        if self._type == 'Position':
            self._speed = self._speed / self.frequency

        self.zero_motion = True

        logger.info('Component initialized')

    def default_action(self):
        """ Interpret keyboard presses and assign them to movement
            for the robot."""
        keyboard = blenderapi.keyboard()
        is_actived = blenderapi.input_active()

        # Reset movement variables
        vx, vy, vz = 0.0, 0.0, 0.0
        rx, ry, rz = 0.0, 0.0, 0.0

        # move forward
        if keyboard.events[blenderapi.UPARROWKEY] == is_actived:
            vx = self._speed

        # move backward
        if keyboard.events[blenderapi.DOWNARROWKEY] == is_actived:
            vx = -self._speed

        # turn left
        if keyboard.events[blenderapi.LEFTARROWKEY] == is_actived:
            rz = self._speed

        # turn right
        if keyboard.events[blenderapi.RIGHTARROWKEY] == is_actived:
            rz = -self._speed

        # move up
        if keyboard.events[blenderapi.TKEY] == is_actived:
            vz = self._speed

        # move down
        if keyboard.events[blenderapi.GKEY] == is_actived:
            vz = -self._speed

        # strafe left
        if keyboard.events[blenderapi.UKEY] == is_actived:
            vy = self._speed

        # strafe right
        if keyboard.events[blenderapi.OKEY] == is_actived:
            vy = -self._speed

        # roll left
        if keyboard.events[blenderapi.JKEY] == is_actived:
            rx = -self._speed

        # roll right
        if keyboard.events[blenderapi.LKEY] == is_actived:
            rx = self._speed

        # pitch forward
        if keyboard.events[blenderapi.IKEY] == is_actived:
            ry = self._speed

        # pitch backward
        if keyboard.events[blenderapi.KKEY] == is_actived:
            ry = -self._speed

        # Send a 'zero motion' only once in a row.
        if self.zero_motion and (vx,vy,vz,rx,ry,rz) == (0,0,0,0,0,0):
            return

        if self._type == 'Position' or self._type == 'Velocity':
            self.robot_parent.apply_speed(self._type, [vx, vy, vz], [rx, ry, rz / 2.0])
        elif self._type == 'Differential':
            self.robot_parent.apply_vw_wheels(vx, rz)


        if (vx,vy,vz,rx,ry,rz) == (0,0,0,0,0,0):
            self.zero_motion = True
        else:
            self.zero_motion = False


