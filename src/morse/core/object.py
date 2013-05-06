import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta, abstractmethod
from collections import OrderedDict
from morse.core.abstractobject import AbstractObject

import morse.helpers.transformation
from morse.core.services import service

from morse.core import blenderapi

class Object(AbstractObject):
    """ Basic Class for all 3D objects (components) used in the simulation.
        Provides common attributes.
    """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self, obj, parent=None):

        super(Object, self).__init__()

        # Fill in the data sent as parameters
        self.bge_object = obj
        self.robot_parent = parent

        self.level = self.bge_object.get("abstraction_level", "default")

        # Variable to indicate the activation status of the component
        self._active = True

        self.check_level()

        # Define the position of sensors with respect
        #  to their robot parent
        # TODO: implement this using morse.helpers.transformation
        if parent:
            self.relative_position = obj.getVectTo(parent.bge_object)

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
        # New MORSE_LOGIC sensor, see AbstractComponent.morseable()
        morselogic = [s for s in sensors if s.name.startswith('MORSE_LOGIC')]
        if len(morselogic) == 1:
            self._frequency /= morselogic[0].frequency + 1
        # Backward compatible (some actuators got special logic)
        elif len(sensors) == 1:
            self._frequency /= sensors[0].frequency + 1
        elif len(sensors) == 0:
            logger.warning("Can't get frequency for " + self.name() + \
                           " as the Game Logic sensor calling the action can't be found.")
        else:
            logger.warning(self.name() + " has too many Game Logic sensors to get " + \
                    "an unambiguous frequency for the action.")

    def check_level(self):

        if self.level == "default":
            return # fine

        if hasattr(self, '_levels') and self.level in self._levels:
            return #fine

        msg = "Component <%s> has no abstraction level <%s>. Please check your scene." % (self.name(), self.level)
        logger.error(msg)
        raise ValueError(msg)


    def initialize_local_data(self):
        """
        Creates and initializes 'local data' fields, according to the
        current component abstraction level.
        """

        if hasattr(self, '_data_fields'):
            for name, details in self._data_fields.items():
                default_value, type, doc, level = details
                if level == "all" or level == self.level:
                    self.local_data[name] = default_value

    def update_properties(self):
        """
        Takes all registered properties (see add_property), and update
        their values according to the values set in Blender object.
        """

        all_properties = OrderedDict()
        for cls in reversed(type(self).__mro__):
            if hasattr(cls, '_properties'):
                all_properties.update(cls._properties)

        for name, details in all_properties.items():
            default_value, _, _, python_name = details
            val = default_value

            try:
                val = self.bge_object[name]
            except KeyError:
                pass
            setattr(self, python_name, val)

    def name(self):
        return self.bge_object.name

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
