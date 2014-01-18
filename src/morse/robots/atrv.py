import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot

class ATRV(morse.core.robot.Robot):
    """
    Definition for the well-known ATRV robot, distributed by I-Robot.
    """

    _name = "iRobot ATRV platform"

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        morse.core.robot.Robot.__init__(self, obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        pass
