import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot


class FakeRobot(morse.core.robot.Robot):
    """
    This is a special case of component in MORSE. Since all sensors or actuators
    must be attached to one robot, it would not normally be possible to use
    "stand-alone" sensors in the environment.

    If you need to use a sensor in this way, (*i.e.* for motion capture sensors,
    or independent cameras) you should add an **virtual (fake)
    robot** to the scene, and make it the parent of your stand-alone
    sensors.

    This robot has no visual representation, and consists of a single Blender
    Empty. Its only purpose is to provide the base to attach sensors. A single
    fake robot can be the parent of as many
    sensors/actuators as needed.
    """

    _name = 'Fake virtual robot'

    def __init__(self, obj, parent=None):
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        morse.core.robot.Robot.__init__(self, obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        pass
