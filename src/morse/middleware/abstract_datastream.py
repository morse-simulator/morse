from abc import ABCMeta, abstractmethod

class AbstractDatastream(object):
    """
    The class is inherited by all serializers/deserializers.
    Concrete classes need to implement :py:meth:`default`.
    """
    _type_name = "(this serializer does not document its output/input type)"
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
        """ initialize the specific datastream

        Can be overriden if needed
        """
        pass

    @abstractmethod
    def default(self, ci='unused'):
        """ default method called by MORSE logic

        Sensor: must read `local_data`, format and publish them.
        Actuator: must read a new incoming command and update `local_data`.
        """
        # :param ci: kept for backward compatibility, unused component_instance
        #            passed from Sensor / Actuator default_action
        pass

    def finalize(self):
        """ finalize the specific datastream

        Can be overriden if needed
        """
        pass

    def __del__(self):
        self.finalize()
