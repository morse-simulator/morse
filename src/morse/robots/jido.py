import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.robot


class JidoClass(morse.core.robot.MorseRobotClass):
    """ Class definition for the Jido robot.

        Sub class of Morse_Object. """

    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            Optionally it gets the name of the object's parent,
            but that information is not currently used for a robot. """
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        super(self.__class__,self).__init__(obj, parent)

        # Add the variable move_status to the object
        self.move_status = "Stop"

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        pass
