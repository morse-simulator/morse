import logging

logger = logging.getLogger("morse." + __name__)
from morse.core.services import service
import morse.core.actuator
from morse.helpers.components import add_data


class MotionVWDiff(morse.core.actuator.Actuator):
    """
    This actuator reads the values of linear and angular speed and
    applies them to the robot as speeds for the wheels. It only works
    with robots of the type ``WheeledRobot``, such as the :doc:`Segway
    RMP 400 <../robots/segwayrmp400>` and the :doc:`Pioneer 3-DX
    <../robots/pioneer3dx>`.  The movement of the robot is more
    realistic, but also depends on more factors, such as the friction
    between the wheels and the surface.

    The speeds for the left and right wheels are calculated as:

            left_speed = (v - e w) / R
            right_speed = (v + e w) / R

    where:

        - **v** is the linear velocity given as parameter
        - **w** is the angular velocity given as parameter
        - **e** is half of the distance between the left and
        right wheels
        - **R** is the radius of the wheels

    """

    _name = 'Differential Driver Actuator: \
            Linear and angular speed (V, W) actuator'

    add_data('v', 0.0, 'float',
            'linear velocity in x direction (forward movement) (m/s)')
    add_data('w', 0.0, 'float', 'angular velocity (rad/s)')

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        # Call the constructor of the parent class
        morse.core.actuator.Actuator.__init__(self, obj, parent)

        self.local_data['v'] = 0.0
        self.local_data['w'] = 0.0

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

        self.robot_parent.apply_vw_wheels(self.local_data['v'],
                                          self.local_data['w'])

