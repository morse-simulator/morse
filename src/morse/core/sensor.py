import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta
import time # profiler
import morse.core.object
from morse.core.services import service

class Sensor(morse.core.object.Object):
    """ Basic Class for all sensors

    Inherits from the base object class.
    """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self, obj, parent=None):
        """ Constructor method. """
        # Call the constructor of the parent class
        super(Sensor, self).__init__(obj, parent)

        # Define lists of dynamically added functions
        self.output_functions = []
        self.output_modifiers = []

        self.profile = None
        if "profile" in self.bge_object:
            self.time = {}
            self.profile = ["profile", "profile_action", "profile_modifiers",
                            "profile_datastreams"]
            for key in self.profile:
                self.time[key] = 0.0
            self.time_start = time.time()

    def finalize(self):
        self._active = False
        super(Sensor, self).finalize()
        del self.output_functions[:]
        del self.output_modifiers[:]

    def sensor_to_robot_position_3d(self):
        """
        Compute the transformation between the sensor and the
        associated robot
        """
        main_to_origin = self.robot_parent.position_3d
        main_to_sensor = main_to_origin.transformation3d_with(self.position_3d)
        return main_to_sensor

    def action(self):
        """ Call the action functions that have been added to the list. """
        # Do nothing if this component has been deactivated
        if not self._active:
            return

        # Update the component's position in the world
        self.position_3d.update(self.bge_object)

        # record the time before performing the default action for profiling
        if self.profile:
            time_before_action = time.time()

        # Call the regular action function of the component
        self.default_action()

        # record the time before calling modifiers for profiling
        if self.profile:
            time_before_modifiers = time.time()

        # Data modification functions
        for function in self.output_modifiers:
            function()

        # record the time before calling datastreams for profiling
        if self.profile:
            time_before_datastreams = time.time()

        # Lastly output functions
        for function in self.output_functions:
            function(self)

        # profiling
        if self.profile:
            time_now = time.time()
            self.time["profile"] += time_now - time_before_action
            self.time["profile_action"] += time_before_modifiers - time_before_action
            self.time["profile_modifiers"] += time_before_datastreams - time_before_modifiers
            self.time["profile_datastreams"] += time_now - time_before_datastreams
            morse_time = time_now - self.time_start
            for key in self.profile:
                ratio = self.time[key] / morse_time
                # format the display
                self.bge_object[key] = "%4.1f%% %s"% (100.0 * ratio, '█' * int(10 * ratio))
            if morse_time > 1: # re-init mean every sec
                for key in self.profile:
                    self.time[key] = 0.0
                self.time_start = time.time()

    @service
    def get_local_data(self):
        """
        Returns the current data stored in the sensor.

        :return: a dictionary of the current sensor's data
        """
        return (self.local_data)
