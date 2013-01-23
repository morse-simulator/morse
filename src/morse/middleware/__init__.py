from abc import ABCMeta, abstractmethod

class AbstractDatastream(object):
    __metaclass__ = ABCMeta

    def __init__(self, component_instance, kwargs):
        self.component_instance = component_instance
        self.kwargs = kwargs
        self.initalize()

    @property
    def component_name(self):
        return self.component_instance.bge_object.name

    def __str__(self):
        return '%s(%s)'%(self.__class__.__name__, self.component_name)

    def initalize(self):
        """ initalize the specific datastream

        Can be overriden if needed
        """
        pass

    @abstractmethod
    def default(self, ci=None):
        """ default method called by MORSE logic

        Sensor: must read `local_data`, format and publish them.
        Actuator: must read a new incoming command and update `local_data`.
        """
        # :param ci: kept for backward compatibility unused
        #   component_instance passed from Sensor / Actuator default_action
        pass

    def finalize(self):
        """ finalize the specific datastream

        Can be overriden if needed
        """
        pass

    def __del__(self):
        self.finalize()
