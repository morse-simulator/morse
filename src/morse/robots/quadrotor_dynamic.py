import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot

class Quadrotor(morse.core.robot.Robot):
    """
    Simple definition of a quadrotor, with ``Rigid Body`` physics.

    It is expected to be used with actuators such as:
        - :doc:`../actuators/force_torque`
        - :doc:`../actuators/rotorcraft_attitude`
        - :doc:`../actuators/rotorcraft_waypoint`
    """

    _name = 'Quadrotor with dynamics'

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        morse.core.robot.Robot.__init__(self, obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        pass
