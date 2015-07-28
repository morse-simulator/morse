import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta, abstractmethod
import morse.core.object
from morse.helpers.components import add_data

class ExternalObject(morse.core.object.Object):
    """ Basic Class for all External Object

    Provides common attributes. """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self, obj, parent=None):
        """ Constructor method. """
        # Call the constructor of the parent class
        morse.core.object.Object.__init__(self, obj, parent)

        # Define lists of dynamically added functions
        self.input_functions = []
        self.output_functions = []
        self.input_modifiers = []
        self.output_modifiers = []

    def finalize(self):
        self._active = False
        morse.core.object.Object.finalize(self)
        del self.input_functions[:]
        del self.output_functions[:]
        del self.input_modifiers[:]
        del self.output_modifiers[:]


    def action(self):
        """ Call the action functions that have been added to the list. """
        # Do nothing if this component has been deactivated
        if not self._active:
            return

        # Update the component's position in the world
        self.position_3d.update(self.bge_object)

        received = False
        status = False

        # First the input functions
        for function in self.input_functions:
            status = function(self)
            received = received or status

        if received:
            # Data modification functions
            for function in self.input_modifiers:
                function()

        # Call the regular action function of the component
        self.default_action()

        # Data modification functions
        for function in self.output_modifiers:
            function()

        # Lastly output functions
        for function in self.output_functions:
            function(self)

class ExternalSensor(ExternalObject):
    add_data('timestamp', 0.0, 'float', 
             'number of seconds in simulated time')

    def action(self):
        self.local_data['timestamp'] = self.robot_parent.gettime()

        ExternalObject.action(self)


class ExternalActuator(ExternalObject):
    pass
