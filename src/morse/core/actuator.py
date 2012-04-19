import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta, abstractmethod
import morse.core.object

class MorseActuatorClass(morse.core.object.MorseObjectClass):
    """ Basic Class for all actuator ovjects.

    Provides common attributes. """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self, obj, parent=None):
        """ Constructor method. """
        # Call the constructor of the parent class
        super(MorseActuatorClass, self).__init__(obj, parent)
        #super(self.__class__, self).__init__(obj, parent)

        # Define lists of dynamically added functions
        self.input_functions = []
        self.input_modifiers = []


    def action(self):
        """ Call the action functions that have been added to the list. """
        # Do nothing if this component has been deactivated
        if not self._active:
            return

        # Update the component's position in the world
        self.position_3d.update(self.blender_obj)

        received = False
        status = False

        # First the input functions
        for function in self.input_functions:
            status = function(self)
            received = received or status

        if received:
            # Data modification functions
            for function in self.input_modifiers:
                function(self)

        # Call the regular action function of the component
        self.default_action()
