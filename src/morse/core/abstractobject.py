import logging; logger = logging.getLogger("morse." + __name__)
from abc import ABCMeta, abstractmethod
import morse.core.services
from collections import OrderedDict

from morse.core.exceptions import MorseRPCInvokationError, MorseServiceAlreadyRunningError

class AbstractObject(object):
    """ An abstract object. All components in MORSE (either physical components
    like actuators or sensors or pseudo components like component overlays) 
    inherit from AbstractObject.
    
    This abstract class provides all RPC service-related mechanics.
    """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__(self):
        
        self.on_completion = None
        """ When a task is considered 'completed' (the semantic of
        'completed' is left to each component), the default_action
        method is expected to call this callback (if not None) with
        the task status (from ``core.status.*``) + optional return value
        as a tuple.

        For instance::

          self.on_completion((status.FAILED, "Couldn't reach the target"))
          self.on_completion((status.SUCCESS, {'x':1.0, 'y':0.54}))
          self.on_completion((status.SUCCESS))
        """
        
        # Dictionary to store the data used by each component
        self.local_data = OrderedDict()
        
        # Define lists of dynamically added functions
        self.del_functions = []
        

    def finalize(self):
        """ Destructor method. """
        logger.info("%s: I'm dying!!" % self.name())
        # Call specific functions added to this object
        for function in self.del_functions:
            function()
    
    def register_services(self):
        """
        Register the component services, if any.
        Methods to register are marked '_morse_service' by the '@service' decorator.
        """
        for fn in [getattr(self, fn) for fn in dir(self) if hasattr(getattr(self, fn), "_morse_service")]:
            name = fn._morse_service_name if fn._morse_service_name else fn.__name__
            morse.core.services.do_service_registration(fn, self.name(), name, fn._morse_service_is_async)
    
    
    def completed(self, status, result = None):
        """ This method must be invoked by the component when a service completes.

        Calling this method triggers the notification of task completion to the client.

        :param morse.core.status status: status (success, failure...) of the task

        :param result: results of the service, if any (may be any valid Python object)

        """
        if self.on_completion:
            """ In overlays or in a chain of service, it is legal to call again a
            service in the completion handler. However, to test if there
            is a running service or not for a service, we test if
            self.on_completion is None or not. So, we need to make sure
            that self.on_completion is None when calling the completion
            handler. To do that, store another reference, clean up the
            one stored in self.on_completion, and call it through our reference
            """
            fn = self.on_completion
            self.on_completion = None
            fn((status, result))

    def interrupt(self):
        """ This method is automatically invoked by the component when a
        service is interrupted, basically to notify to the client that
        the task has been interrupted.

        It can be overriden in each component to provide a true
        interrupt, for exemple resseting the speed command to 0.0.

        If you override it, do not forget to call ``self.completed`` with
        ``status.PREEMPTED``.

        """
        import morse.core.status

        self.completed(morse.core.status.PREEMPTED)

    def set_service_callback(self, cb):
        """ Sets the callback function that is to be invoked when the current
        request completes.

        This is automatically set by the @async_service decorator and should
        not usually be directly called.
        """

        if self.on_completion: # A callback is already registered -> a service is running
            old_cb = self.on_completion
            if not hasattr(old_cb.service, "_morse_service_interruptible"):
                # No policy (interruptible/noninterruptible) set for
                # the service currently running. We throw an exception 
                # to be caught by the middleware. Up to it to define 
                # the default policy.
                raise MorseServiceAlreadyRunningError(old_cb.service, "A service (" + \
                        old_cb.service.__name__ + ") is already running. I do not know what to do.")

            interruptible = old_cb.service._morse_service_interruptible

            if interruptible:
                self.interrupt()
            else:
                raise MorseRPCInvokationError("A non-interruptible service (" + old_cb.service.__name__ + ") is already running")

        self.on_completion = cb
    
    @abstractmethod
    def name(self):
        """ The name of the component, as appearing when exposing services for
        instance.
        """
        pass
    
    def print_data(self):
        """ Print the current data for the component instance. """
        for variable, data in self.local_data.items():
            res = variable + str(data) + " "
        logger.info("%s" % res)
