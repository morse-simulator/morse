from abc import ABCMeta, abstractmethod

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
        return self.component_instance.bge_object.name

    @property
    def data(self):
        return self.component_instance.local_data

    def __str__(self):
        return '%s(%s)'%(self.__class__.__name__, self.component_name)

    def initialize(self):
        """ initialize the specific modifier

        Can be overriden if needed
        """
        pass

    @abstractmethod
    def modify(self):
        """ default method called by MORSE logic
        """
        pass

    def finalize(self):
        """ finalize the specific modifier

        Can be overriden if needed
        """
        pass

    def __del__(self):
        self.finalize()
