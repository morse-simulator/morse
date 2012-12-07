import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta
import time # profiler
import morse.core.object
from morse.core.services import service
from morse.core import blenderapi

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
        #super(self.__class__, self).__init__(obj, parent)

        # Define lists of dynamically added functions
        self.output_functions = []
        self.output_modifiers = []

        self.my_time = 0.0
        self.start_time = time.time()

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
        self.position_3d.update(self.blender_obj)

        # record the time before performing the default action for profiling
        time_before_action = time.time()

        # Call the regular action function of the component
        self.default_action()

        # profiling TODO ? put this in morse.core.object.Object.action ?
        # or move this code at the end of the method to count datastream
        # time as well, or profile it separately +property Component.datastream
        if self.name() in blenderapi.getssr():
            time_now = time.time()
            self.my_time += time_now - time_before_action
            morse_time = time_now - self.start_time
            ratio = 100.0 * self.my_time / morse_time
            blenderapi.getssr()[self.name()] = "%6.2f %%"%ratio
            if morse_time > 1: # re-init mean every sec
                self.my_time = 0.0
                self.start_time = time_now

        # Data modification functions
        for function in self.output_modifiers:
            function(self)

        # Lastly output functions
        for function in self.output_functions:
            function(self)

    @service
    def get_local_data(self):
        """Returns the current data stored in the sensor.

        :return: a dictionary of the current sensor's data
        """
        return (self.local_data)
