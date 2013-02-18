import logging; logger = logging.getLogger("morse." + __name__)
# Modules necessary to dynamically add methods to Middleware subclasses
import os
import sys
import re
import types

from abc import ABCMeta, abstractmethod

from morse.core.sensor import Sensor
from morse.core.actuator import Actuator
from morse.middleware import AbstractDatastream
from morse.helpers.loading import create_instance, load_module_attribute


def register_datastream(classpath, component, args):
    datastream = create_instance(classpath, component, args)
    # Check that datastream implements AbstractDatastream
    if not isinstance(datastream, AbstractDatastream):
        logger.warning("%s should implement morse.middleware.AbstractDatastream"%classpath)
    # Determine weither to store the function in input or output list,
    #   what is the direction of our stream?
    if isinstance(component, Sensor):
        # -> for Sensors, they *publish*,
        component.output_functions.append(datastream.default)
    elif isinstance(component, Actuator):
        # -> for Actuator, they *read*
        component.input_functions.append(datastream.default)
    else:
        logger.error("The component is not an instance of Sensor or Actuator")
        return None
    # from morse.core.abstractobject.AbstractObject
    component.del_functions.append(datastream.finalize)

    return datastream


class Datastream(object):
    """ Basic Class for all middlewares

    Provides common attributes. """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self):
        """ Constructor method. """
        self._extra_methods = []

    def __del__(self):
        """ Destructor method. """
        logger.info("Closing datastream interface <%s>." % self.__class__.__name__)


    @abstractmethod
    def register_component():
        """ Abstract model for the component binding method

        Implemented by all subclasses of Datastream.
        """
        
        
        pass


    def cleanup(self):
        """ Remove the modules linked dynamically """
        for module in self._extra_methods:
            delattr(self, module)


    def _check_function_exists(self, function_name):
        """ Checks if a class contains a specified function
        
        Returns a reference to the function, that can be used
        by other components
        """
        try:
            # Get the reference to the function
            function = getattr(self, function_name)
            return function
        except AttributeError as detail:
            #logger.error("while checking middleware functions: %s\nCheck the 'component_config.py' file for typos or contact the component developer." % detail)
            return None


    def _add_method(self, mw_data, component_instance):
        """ Include a new serialisation method in the middleware.

        Parameters are the array with parameters for the middleware
        and the instance of the component
        """
        # Get the variables from the data passed
        # Second parameter is the name of the function to add to the component
        function_name = mw_data[1]
        # Third parameter is the file where the function can be found
        source_file = mw_data[2]

        module_name = re.sub('/', '.', source_file)
        # Import the module containing the class
        func = load_module_attribute(module_name, function_name)
        if not func:
            logger.error("Check the 'component_config.py' file for typos.")
            return

        # Insert the function and get a reference to it
        setattr(self, func.__name__, types.MethodType(func, self))

        bound_function = getattr(self, function_name)
        try:
            # Call the init method of the new serialisation
            # Sends the name of the function as a means to identify
            #  what kind of port it should use.
            module.init_extra_module(self, component_instance, bound_function, mw_data)
        except AttributeError as detail:
            logger.error("%s in module '%s'" % (detail, source_file))

        # Store the name of the function, to cleanup later
        # If function with the same name already included, pass. Otherwise middleware will fail to cleanup
        if function_name in self._extra_methods:
            logger.info("Extra method already known")	
        else:
            self._extra_methods.append(function_name)

        return bound_function
