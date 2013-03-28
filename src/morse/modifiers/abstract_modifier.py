from abc import ABCMeta, abstractmethod

from morse.core import blenderapi

class AbstractModifier(object):
    """
    The class is inherited by all modifiers.
    Concrete classes need to implement :py:meth:`modify`.
    """
    _type_name = "(this modifier does not document its output/input type)"
    _type_url = ""

    __metaclass__ = ABCMeta

    def __init__(self, component_instance, kwargs):
        self.component_instance = component_instance
        self.kwargs = kwargs
        self.initialize()

    @property
    def component_name(self):
        """ get the component name
        """
        return self.component_instance.bge_object.name

    @property
    def data(self):
        """ get the component local data
        """
        return self.component_instance.local_data
    
    def parameter(self, arg, prop=None, default=None):
        """ get a modifier parameter
        
        The parameter is (by priority order):
        1. get from modifier kwargs
        2. get from the scene properties
        3. set to None 
        """
        ssr = blenderapi.getssr()
        if arg in self.kwargs:
            return self.kwargs[arg]
        else:
            try:
                if prop:
                    x = ssr[prop]
                else:
                    x = ssr[arg]
                return x
            except KeyError:
                return default

    def __str__(self):
        return '%s(%s)'%(self.__class__.__name__, self.component_name)
    
    def key_error(self, detail):
        """ logs a standard warning if some data has not been found in the component
        """ 
        logger.warning("Unable to use %s on component %s. It does not have data %s." 
                       % (self.__class__.__name__, self.component_name, detail))

    def initialize(self):
        """ initialize the specific modifier

        Can be overridden if needed
        """
        pass

    @abstractmethod
    def modify(self):
        """ default method called by MORSE logic
        
        All modifiers have to override this method.
        """
        pass

    def finalize(self):
        """ finalize the specific modifier
        
        Can be overridden if needed
        """
        pass

    def __del__(self):
        self.finalize()
