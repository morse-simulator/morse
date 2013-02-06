import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
import morse.core.actuator
from morse.helpers.components import add_data, add_property

class Keyboard(morse.core.actuator.Actuator):
    """
    This actuator does not require a connection with external data. It
    simply responds to the keyboard arrows to generate movement
    instructions for the robot attached.

    When parented to a robot, the user can press the arrow keys to modify the
    linear and angular velocities (V, W) of the robot.

    :kbd:`Up` forward
    :kbd:`Down` backwards
    :kbd:`Left` turn left
    :kbd:`Right` turn right
    """

    _name = "Keyboard Actuator"
    _short_desc="A 'fake' actuator that allows to move a robot from the keyboard."

    add_property('_speed', 1.0, 'Speed', 'float',
                 "Movement speed of the parent robot, in m/s")

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)


        # Choose the type of function to move the object
        #self._type = 'Velocity'
        self._type = 'Position'

        # Correct the speed considering the Blender clock
        if self._type == 'Position':
            self._speed = self._speed / self.frequency

        logger.info('Component initialized')



    def default_action(self):
        """ Interpret keyboard presses and assign them to movement
            for the robot."""
        keys_sensor = blenderapi.controller().sensors[0]
        #pressed_keys = keys_sensor.getPressedKeys()
        pressed_keys = keys_sensor.events

        # Reset movement variables
        vx, vy, vz = 0.0, 0.0, 0.0
        rx, ry, rz = 0.0, 0.0, 0.0

        for key, status in pressed_keys:
            logger.debug("GOT: {0}, STATUS {1}".format(key, status))
            if key == blenderapi.UPARROWKEY:
                vx = self._speed

            if key == blenderapi.DOWNARROWKEY:
                vx = -self._speed

            if key == blenderapi.LEFTARROWKEY:
                rz = self._speed / 2.0

            if key == blenderapi.RIGHTARROWKEY:
                rz = -self._speed / 2.0

        # Get the Blender object of the parent robot
        parent = self.robot_parent.bge_object

        # Give the movement instructions directly to the parent
        # The second parameter specifies a "local" movement
        if self._type == 'Position':
            parent.applyMovement([vx, vy, vz], True)
            parent.applyRotation([rx, ry, rz], True)
        elif self._type == 'Velocity':
            parent.setLinearVelocity([vx, vy, vz], True)
            parent.setAngularVelocity([rx, ry, rz], True)
