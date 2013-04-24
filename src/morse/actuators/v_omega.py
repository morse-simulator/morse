import logging; logger = logging.getLogger("morse." + __name__)
from morse.core.services import service
import morse.core.actuator
from morse.helpers.components import add_data, add_property

class MotionVW(morse.core.actuator.Actuator):
    """ 
    This actuator reads the values of linear and angular speed and
    applies them to the robot as direct translation. The speeds provided
    are internally adjusted to the Blender time measure.
    """
    _name = 'Linear and angular speed (V, W) actuator'
    _short_desc = 'Motion controller using linear and angular speeds'

    add_data('v', 0.0, 'float',
             'linear velocity in x direction (forward movement) (m/s)')
    add_data('w', 0.0, 'float', 'angular velocity (rad/s)')

    add_property('_type', 'Position', 'ControlType', 'string',
                 "Kind of control, can be one of ['Velocity', 'Position']")

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        super(self.__class__, self).__init__(obj, parent)

        logger.info('Component initialized')


    @service
    def set_speed(self, v, w):
        """
        Modifies v and w according to the parameters

        :param v: desired linear velocity (meter by second)
        :param w: desired angular velocity (radian by second)
        """
        self.local_data['v'] = v
        self.local_data['w'] = w

    @service
    def stop(self):
        """
        Stop the robot

        Internally, it sets (v, w) to (0.0, 0.0)
        """
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

        self.apply_speed(self._type, [vx, vy, vz], [rx, ry, rz])
