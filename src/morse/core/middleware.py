import logging; logger = logging.getLogger("morse." + __name__)
# Modules necessary to dynamically add methods to Middleware subclasses
import os
import sys
import re
import types

from abc import ABCMeta, abstractmethod

class MorseMiddlewareClass(object):
    """ Basic Class for all middlewares

    Provides common attributes. """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self, obj, parent=None):
        """ Constructor method. """
        self.blender_obj = obj
        self._extra_methods = []

    def __del__(self):
        """ Destructor method. """
        logger.info("%s: Middleware finishing" % self.blender_obj.name)


    @abstractmethod
    def register_component():
        """ Abstract model for the component binding method

        Implemented by all subclasses of MorseMiddlewareClass.
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
        try:
            __import__(module_name)
        except ImportError as detail:
            logger.error("%s. Check the 'component_config.py' file for typos" % (detail))
            return
        module = sys.modules[module_name]

        try:
            # Get the reference to the new method
            func = getattr(module, function_name)
        except AttributeError as detail:
            logger.error("%s, in extra module '%s'. Check the 'component_config.py' file for typos" % (detail, module_name))
            return

        # Insert the function and get a reference to it
        if sys.version_info >= (3,0,0):
            # NOTE (GEF 02/11/2010): I don't know why the arguments changed
            #  in Python 3, and not sure this will work, but it seems to. :-)
            setattr(self, func.__name__, types.MethodType(func, self))
        else:
            setattr(self, func.__name__, types.MethodType(func, self, self.__class__))

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
