import logging; logger = logging.getLogger("morse." + __name__)
logger.setLevel(logging.DEBUG)
from abc import ABCMeta, abstractmethod
from functools import partial
from morse.core.abstractobject import AbstractObject
from morse.core.exceptions import MorseRPCInvokationError

class MorseOverlay(AbstractObject):
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

    def _chain_callback(self, fn, result):
        logger.debug("Calling " + self.name() + " chain callback")

        if fn:
            result = fn(result)

        self.on_completion(result)
        self.on_completion = None

    def chain_callback(self, fn = None):
        """ When calling a component asynchronous service from
        an overlay, a callback (used to notify the client upon
        service completion) must be passed through. This callback
        does not usually appear in the service signature since it
        is added by the ``@async_service`` decorator.

        Under normal circumstances, you must use this method as
        callback.

        For instance, assume a ``Dummy`` component and an overlay
        ``MyDummy``:

        .. code-block:: python

            class Dummy(MorseObject):
                @async_service
                def dummy_service(self, arg1):
                    # Here, dummy_service has a callback parameter added
                    # by the decorator
                    pass

            class MyDummy(MorseAbstractobject):
                @async_service
                def mydummy_service(self, arg1):
                    # [...do smthg useful]

                    # We call the overlaid asynchronous service
                    # 'dummy_service' by passing a special callback
                    # returned by 'self.chain_callback()'
                    self.overlaid_object.dummy_service(self.chain_callback(), arg1)

        ``chain_callback`` takes a functor as an optional parameter.
        This functor is called after the (overlaid) service completion, but
        just before notifying the simulator client.

        It can be used for output formatting for instance.

        The functor *must* take one single parameter (a tuple ``(status, result)``)
        and must as well return a tuple ``(status, result)``.

        .. code-block:: python

            class MyDummy(MorseAbstractobject):

                def mydummy_on_completion(self, result):
                    # This functor - here a simple function - simply
                    # format the result output.
                    # It could do anything else.
                    status, value = result
                    return (status, " . ".join(value))

                @async_service
                def mydummy_service(self, arg1):
                    self.overlaid_object.dummy_service(self.chain_callback(self.mydummy_on_completion), arg1)

        :param fn: a functor to be called on service completion, before 
                    notifying the clients. Must have the following signature:
                    (status, result) fn((status, result))

        """
        return partial(self._chain_callback, fn)
        

    def name(self):
        """ Returns the overlaid component name.
        
        By default, the name of the class of the overlaid component.
        
        Override this method in your overlay to expose an alternative name.
        """
        return self.overlaid_object.name()

    def interrupt(self):
        if self.overlaid_object.on_completion:
            self.overlaid_object.interrupt()
        else:
            super(MorseOverlay, self).interrupt()
