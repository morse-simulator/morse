import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot


class Environment(morse.core.robot.Robot):
    """
    This is a special case of component in MORSE. Since all sensors or actuators
    must be attached to one robot, it would not normally be possible to use
    "stand-alone" sensors in the environment.

    If you need to use a sensor in this way, (*i.e.* for motion capture sensors,
    or independent cameras) you should add an **environment virtual
    robot** to the scene, and make it the parent of your stand-alone
    sensors.

    This robot has no visual representation, and consists of a single Blender
    Empty. Its only purpose is to provide the base to attach sensors. A single
    **environment virtual robot** should be the parent of as many
    sensors as needed.
    """

    _name = 'Environment virtual robot'

    def __init__(self, obj, parent=None):
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        super(self.__class__, self).__init__(obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        pass
