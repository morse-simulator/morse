import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot

class Morsy(morse.core.robot.Robot):
    """
    Morsy is the little mascot of the MORSE project.

    The model does not feature any particular behaviour. It's main
    purpose is for teaching and examples.
    """

    _name = 'The MORSE Morsy mascot'

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        morse.core.robot.Robot.__init__(self, obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        pass
