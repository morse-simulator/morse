from abc import ABCMeta, abstractmethod
import morse.helpers.object

class MorseRobotClass(morse.helpers.object.MorseObjectClass):
    """ Basic Class for all robots

    Inherits from the base object class.
    """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self, obj, parent=None):
        """ Constructor method. """
        # Call the constructor of the parent class
        super(MorseRobotClass, self).__init__(obj, parent)
        #super(self.__class__, self).__init__(obj, parent)

        # Add the variable move_status to the object
        self.move_status = "Stop"


    def action(self):
        """ Call the regular action function of the component. """
        self.default_action()
