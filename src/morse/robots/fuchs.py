import logging; logger = logging.getLogger("morse." + __name__)
import bge
import morse.core.robot


class FuchsClass(morse.core.robot.MorseRobotClass):
    """ Class definition for the FUCHS.
        Sub class of Morse_Object.
    
        Fuchs if segway like robot used by Intelligent Industrial Robots group (IIROB)
        on Institue for Process Control and Robotics (IPR) of Karlsruhe Institue of Technlogy.
        http://rob.ipr.kit.edu/english/303.php
    """
        
    def __init__(self, obj, parent=None):
        """ Constructor method.
            Receives the reference to the Blender object.
            Optionally it gets the name of the object's parent,
            but that information is not currently used for a robot. """
        # Call the constructor of the parent class
        logger.info('%s initialization' % obj.name)
        super(self.__class__,self).__init__(obj, parent)

        logger.info('Component initialized')

    def default_action(self):
        """ Main function of this component. """
        pass
