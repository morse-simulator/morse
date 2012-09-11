import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta, abstractmethod
from morse.core.abstractobject import MorseAbstractObject

import morse.helpers.transformation
from morse.core.services import service

from morse.core import blenderapi

class MorseObjectClass(MorseAbstractObject):
    """ Basic Class for all 3D objects (components) used in the simulation.
        Provides common attributes.
    """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self, obj, parent=None):

        super(MorseObjectClass, self).__init__()

        # Fill in the data sent as parameters
        self.blender_obj = obj
        self.robot_parent = parent

        # Variable to indicate the activation status of the component
        self._active = True

        # Define the position of sensors with respect
        #  to their robot parent
        # TODO: implement this using morse.helpers.transformation
        if parent:
            self.relative_position = obj.getVectTo(parent.blender_obj)

        # Create an instance of the 3d transformation class
        self.position_3d = morse.helpers.transformation.Transformation3d(obj)

        self.initialize_local_data()
        self.update_properties()

        # The actual frequency at which the action is called
        # The frequency of the game sensor specifies how many times
        # the action is skipped when the logic brick is executed.
        # e.g. game sensor frequency = 0 -> sensor runs at full logic rate
        sensors = blenderapi.getalwayssensors(obj)
        self._frequency = blenderapi.getfrequency()
        if len(sensors) == 1:
            self._frequency /= sensors[0].frequency + 1
        elif len(sensors) == 0:
            logger.warning("Can't get frequency for " + self.name() + \
                           " as the Game Logic sensor calling the action can't be found.")
        else:
            logger.warning(self.name() + " has too many Game Logic sensors to get " + \
                    "an unambiguous frequency for the action.")

    def __del__(self):
        """ Destructor method. """
        logger.info("%s: I'm dying!!" % self.name())

    def initialize_local_data(self):

        if hasattr(self, '_data_fields'):
            for name, details in self._data_fields.items():
                default_value, type, doc = details
                self.local_data[name] = default_value

    def update_properties(self):
        """
        Takes all registered properties (see add_property), and update
        their values according to the values set in Blender object.
        """

        if hasattr(self, '_properties'):
            for name, details in self._properties.items():
                default_value, type, doc, python_name = details
                val = default_value

                try:
                    val = self.blender_obj[name]
                except KeyError:
                    pass
                setattr(self, python_name, val)

    def name(self):
        return self.blender_obj.name

    def action(self):
        """ Call the regular action function of the component.

        Can be redefined in some of the subclases (sensor and actuator).
        """
        self.default_action()

    @abstractmethod
    def default_action(self):
        """ Base action performed by any object.

        This method should be implemented by all subclasses that
        will be instanced (GPS, v_Omega, ATRV, etc.).
        """
        pass

    def toggle_active(self):
        self._active = not self._active

    @property
    def frequency(self):
        """ Frequency of the object action in Hz (float). """
        return self._frequency
