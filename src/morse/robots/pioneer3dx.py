import logging; logger = logging.getLogger("morse." + __name__)
import morse.core.wheeled_robot

class Pioneer3DXClass(morse.core.wheeled_robot.MorsePhysicsRobotClass):
    """ Class definition for the Pioneer 3DX base.
        Sub class of Morse_Object. """
              
    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            Optionally it gets the name of the object's parent,
            but that information is not currently used for a robot. """
        logger.info('%s initialization' % obj.name)

        # Call the constructor of the parent class
        super(self.__class__,self).__init__(obj, parent)

        logger.info('Component initialized')


    def default_action(self):
        """ Main function of this component. """
        pass
