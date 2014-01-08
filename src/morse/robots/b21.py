import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot

class B21(morse.core.robot.Robot):
    """
    Definition for the B21 platform, distributed by RWI. This
    cylindrical robot for Human-Robot interactions.
    """

    _name = 'RWI B21 platform'

    def __init__(self, obj, parent=None):
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        morse.core.robot.Robot.__init__(self, obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        pass
