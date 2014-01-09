import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot

class Jido(morse.core.robot.Robot):
    """
    Definition of the very specific LAAS robot Jido. It is built on a NeoBotix base.
    """

    _name = 'LAAS Jido robot'

    def __init__(self, obj, parent=None):
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        morse.core.robot.Robot.__init__(self, obj, parent)

        # Add the variable move_status to the object
        self.move_status = "Stop"

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        pass
