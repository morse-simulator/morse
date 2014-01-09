import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.wheeled_robot

class PatrolBot(morse.core.wheeled_robot.MorsePhysicsRobot):
    """
    The Research PatrolBot, developed by MobileRobots, is a
    differential-drive robot to carry payloads and sensors over all
    normal indoor surfaces in wheelchair-accessible facilities. The
    PatrolBot is an all-purpose indoor base, it can travel at speeds up
    to 2 m/s, and can carry up to 40kg over flat surfaces.
    """

    _name = 'PatrolBot robot'

    def __init__(self, obj, parent=None):
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        morse.core.wheeled_robot.MorsePhysicsRobot.__init__(self, obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        pass
