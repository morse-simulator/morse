import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta, abstractmethod
from morse.core.abstractobject import MorseAbstractObject

from morse.core.exceptions import MorseRPCInvokationError

class MorseOverlay(MorseAbstractObject):
    """ This class allows to define 'overlay'. An 'overlay' is a pseudo component
    that masks a MORSE default component behind a custom facet (with for instance
    custom signatures for services, ports, etc.).
    
    This is especially useful when integrating MORSE into an existing robotic
    architecture where components have custom services/ports/topics whose 
    signature does not match MORSE defaults.
    
    As of MORSE 0.4, only services can currently be overlaid.
    """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self, overlaid_object):
        
        super(MorseOverlay,self).__init__()
        
        # Fill in the data sent as parameters
        self.overlaid_object = overlaid_object
        
        if not self.overlaid_object:
            logger.critical("[INTERNAL ERROR] An overlay can not be initialized before " + \
            "the component it overlays!")
    
    def name(self):
        """ Returns the overlaid component name.
        
        By default, the name of the class of the overlaid component.
        
        Override this method in your overlay to expose an alternative name.
        """
        return self.overlaid_object.name()
