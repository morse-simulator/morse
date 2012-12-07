import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import service
import bge
import morse.core.actuator

class VWActuatorClass(morse.core.actuator.Actuator):
    """ Motion controller using linear and angular speeds

    This class will read linear and angular speeds (V, W)
    as input from an external middleware, and then apply them
    to the parent robot.
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        self.local_data['v'] = 0.0
        self.local_data['w'] = 0.0

        # Choose the type of function to move the object
        #self._type = 'Velocity'
        self._type = 'Position'

        logger.info('Component initialized')


    @service
    def set_speed(self, v, w):
        self.local_data['v'] = v
        self.local_data['w'] = w

    @service
    def stop(self):
        self.local_data['v'] = 0.0
        self.local_data['w'] = 0.0

    def default_action(self):
        """ Apply (v, w) to the parent robot. """

        # Reset movement variables
        vx, vy, vz = 0.0, 0.0, 0.0
        rx, ry, rz = 0.0, 0.0, 0.0

        # Scale the speeds to the time used by Blender
        try:
            if self._type == 'Position':
                vx = self.local_data['v'] / self.frequency
                rz = self.local_data['w'] / self.frequency
            elif self._type == 'Velocity':
                vx = self.local_data['v']
                rz = self.local_data['w']
        # For the moment ignoring the division by zero
        # It happens apparently when the simulation starts
        except ZeroDivisionError:
            pass

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
