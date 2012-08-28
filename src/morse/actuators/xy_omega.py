import logging; logger = logging.getLogger("morse." + __name__)
import bge
import math
import morse.core.actuator

class XYWActuatorClass(morse.core.actuator.MorseActuatorClass):
    """ Motion controller using linear and angular speeds

    This class will read two linear and one angular speeds (Vx, Vy, W)
    as input from an external middleware, and then apply them
    to the parent robot.
    """

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        self.local_data['x'] = 0.0
        self.local_data['y'] = 0.0
        self.local_data['w'] = 0.0

        logger.info('Component initialized')



    def default_action(self):
        """ Apply (x, y, w) to the parent robot. """

        # Reset movement variables
        vx, vy, vz = 0.0, 0.0, 0.0
        rx, ry, rz = 0.0, 0.0, 0.0

        # Scale the speeds to the time used by Blender
        try:
            vx = self.local_data['x'] / self.frequency
            vy = self.local_data['y'] / self.frequency
            rz = self.local_data['w'] / self.frequency

        # For the moment ignoring the division by zero
        # It happens apparently when the simulation starts
        except ZeroDivisionError:
            pass
        # Get the Blender object of the parent robot
        parent = self.robot_parent.blender_obj

        # Give the movement instructions directly to the parent
        # The second parameter specifies a "local" movement
        parent.applyMovement([vx, vy, vz], True)
        parent.applyRotation([rx, ry, rz], True)
        
