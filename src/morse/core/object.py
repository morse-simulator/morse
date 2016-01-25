import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta, abstractmethod
from collections import OrderedDict
from morse.core.abstractobject import AbstractObject
from morse.core.exceptions import MorseRPCInvokationError

import morse.helpers.transformation
from morse.core.services import service

from morse.core import blenderapi

import time
from copy import copy
from morse.core.morse_time import time_isafter

class Object(AbstractObject):
    """ Basic Class for all 3D objects (components) used in the simulation.
        Provides common attributes.
    """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self, obj, parent=None):

        AbstractObject.__init__(self)

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

        if 'frequency' in self.bge_object:
            self._frequency = self.bge_object['frequency']
            self._last_call = None
            self._component_period = 1.0 / self._frequency
            self.__time = blenderapi.persistantstorage().time

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

        all_data_fields = OrderedDict()

        for cls in reversed(type(self).__mro__):
            if hasattr(cls, '_data_fields'):
                all_data_fields.update(cls._data_fields)

        for name, details in all_data_fields.items():
            default_value, type_, doc, level = details
            if level == "all" or self.level in level:
                self.local_data[name] = default_value


    def fetch_properties(self):
        """
        Returns the "_properties" of a component
        :return: a dictionary of the field "_properties"
        """
        all_properties = OrderedDict()

        #fetches '_properties'
        for cls in reversed(type(self).__mro__):
            if hasattr(cls, '_properties'):
                all_properties.update(cls._properties)

        return all_properties

    @service
    def get_properties(self):
        """     
        Returns the properties of a component.

        :return: a dictionary of the current component's properties

        """
        all_properties = self.fetch_properties()
        for prop in all_properties.items():
            l = list(prop[1])
            l[0] = getattr(self, l[3])
            all_properties[prop[0]] = tuple(l)

        return {'properties': all_properties}

    @service
    def set_property(self, prop_name, prop_val):
        """
        Modify one property on a component

        :param prop_name: the name of the property to modify (as shown
        the documentation)
        :param prop_val: the new value of the property. Note that there
        is no checking about the type of the value so be careful

        :return: nothing
        """
        props = self.fetch_properties()
        if prop_name in props:
            setattr(self, props[prop_name][3], prop_val)
        else:
            raise MorseRPCInvokationError(
            "Property %s does not exist for object %s" %
            (prop_name, self.name()))

    @service
    def get_configurations(self):
        """     
        Returns the configurations of a component (parsed from the properties).

        :return: a dictionary of the current component's configurations

        """
        all_properties = self.fetch_properties()
        tmp = {}
        #parses 'all_properties' to get only "key"-"value"-pairs
        #"key" is python_name and "value" is default_value
        for item in all_properties.items():
            tmp[item[0]] = getattr(self, item[1][3])
        transform = self.robot_parent.position_3d.transformation3d_with(self.position_3d)
        rotation = [ list(vec) for vec in transform.rotation_matrix ]
        translation = list(transform.translation)
        tmp['object_to_robot'] = {'rotation': rotation, 'translation': translation}
        return {'configurations': tmp}


    def update_properties(self):
        """
        Takes all registered properties (see add_property), and update
        their values according to the values set in Blender object.
        """

        all_properties = self.fetch_properties()

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

    def periodic_call(self):
        """
        Return true if the component must be called on this loop call, False otherwise
        """
        # must be called on each loop occurence
        if not hasattr(self, '_component_period'):
            return True

        # First call
        if  not self._last_call:
            self._last_call = self.__time.time
            self._nb_call = 1
            return True
        else:
            """
            We deal with a complete simulated second to deal with
            frequencies that are not a fraction of the main simulator frequency.
            For that purpose, we integrate until self._nb_call ==
            self._frequency, i.e.  self._nb_call * self._component_period == 1.0
            """
            must_call = time_isafter(self.__time.time, self._last_call + self._nb_call * self._component_period)
            if must_call:
                if (self._nb_call == self._frequency):
                    self._nb_call = 1
                    self._last_call = copy(self.__time.time)
                else:
                    self._nb_call += 1
            return must_call

    def action(self):
        """ Call the regular action function of the component.

        Can be redefined in some of the subclases (sensor and actuator).
        """
        self.default_action()

    def in_zones(self, name = None, type = None):
        """
        Determine which zone(s) contain(s) current object

        If a :param name: is precised, check only if this specific zone
        contains the position
        If a :param type: is precised, only search in the zone of this
        type.
        """
        zone_manager = blenderapi.persistantstorage().zone_manager
        return zone_manager.contains(self, name = name, type = type)

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
