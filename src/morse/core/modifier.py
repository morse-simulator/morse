import logging; logger = logging.getLogger("morse." + __name__)
# Modules necessary to dynamically add methods to Middleware subclasses
import os
import sys
import re
import types

from abc import ABCMeta, abstractmethod

class Modifier(object):
    """ Basic Class for all modifiers

    Provides common attributes. """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self):
        """ Constructor method. """
        self.initialize()
        
    def __del__(self):
        """ Destructor method. """
        self.finalize()
        
    def initialize(self):
        pass
        
    def finalize(self):
        pass

    @abstractmethod
    def register_component(self, component_name, component_instance, mod_data):
        """ Abstract model for the component binding method
        """
        pass
