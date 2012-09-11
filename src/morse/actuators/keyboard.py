import logging; logger = logging.getLogger("morse." + __name__)
from morse.core import blenderapi
import morse.core.actuator
from morse.helpers.components import add_data, add_property

class KeyboardActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Class definition for a motion controller that changes speed
        and rotation of the robot using the keyboard arrows. """

    add_property('_speed', 1.0, 'Speed', 'float')

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)


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
        keys_sensor = blenderapi.controller().sensors['keys_sensor']
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
        parent = self.robot_parent.blender_obj

        # Give the movement instructions directly to the parent
        # The second parameter specifies a "local" movement
        if self._type == 'Position':
            parent.applyMovement([vx, vy, vz], True)
            parent.applyRotation([rx, ry, rz], True)
        elif self._type == 'Velocity':
            parent.setLinearVelocity([vx, vy, vz], True)
            parent.setAngularVelocity([rx, ry, rz], True)
