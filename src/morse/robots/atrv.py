import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot

class ATRV(morse.core.robot.Robot):
    """
    Definition for the well-known ATRV robot, distributed by I-Robot.
    """

    _name = "iRobot ATRV platform"

    def __init__(self, obj, parent=None):
        logger.info('%s initialization' % obj.name)
        super(self.__class__, self).__init__(obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        pass
