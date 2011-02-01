import os
import sys

from abc import ABCMeta, abstractmethod

class MorseServiceError(Exception):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)

class MorseRPCInvokationError(MorseServiceError):
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)
 

class RequestManager(object):
    """ Basic Class for all request dispatchers, ie, classes that
    implement a 'request service'.

    A request service offers typically 2 things:
        - the ability for a component (a robot, a sensor or the
        simulator as a whole) to expose a RPC method (typically for
        remote configuration or debug),
        - an interface with a specific middleware to serialize the 
        RPC call and communicate with the outside world.

    Components can register such a service with the 'register_service'
    method. Please check its documentation for details.
 
    To implement a concrete RequestManager (for a new middleware, for instance)
    , the following methods must be overloaded:
        - py:meth:_initialization: perform here middleware specific initialization
        - py:meth:_finalization: perform here middleware specific finalization
        - :py:meth:_post_registration: put here all middleware specific code
        that must be executed when a new service is registered.
        - :py:meth:_on_service_completion: this method is called when a 'long term'
        request completes. You should implement here a way to notify
        your clients.
        - :py:meth:_main: this method is called at each step of the
          simulation. You should read there incoming requests and write back
          results.
          When a new request arrives, you must pass it to :py:meth:_on_incoming_request
          that dispatch or invoke properly the request.

    Subclasses are also expected to overload the special :py:meth:__str__
    method to provide middleware specific names.

   """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self):
        """ Constructor method.
        """

        # This map holds the list of all registered services
        # It associates the service name to a tuple
        # (rpc_callable_method, is_done_callable | None)
        self._services = {}

        # This map contains the list of pending requests.
        # It is updated on each call to 
        # :py:meth:update_pending_calls()
        self._pending_calls = {}

        if not self._initialization():
            raise MorseServiceError("Couldn't create the service manager! Initialization failure")

    @abstractmethod
    def _initialization(self):
        """This method is meant to be overloaded by middlewares to perform
        specific initializations.

        Must return True is the initialization is successful, False in other cases.
        """
        pass

    @abstractmethod
    def _finalization(self):
        """This method is meant to be overloaded by middlewares to perform
        specific finalizations.

        Must return True is the finalization is successful, False in other cases.
        """
        pass

    def __del__(self):
        """ Destructor method. """
        if not self._finalization():
            print("WARNING: finalization of the service manager did not complete successfully!")
        
        print ("%s: service manager closed." % self)


    def __str__(self):
        """ Should be overloaded by subclasses to help debug request handling
        for each middleware implementation.
        """
        return "Generic request manager"

    @abstractmethod
    def _post_registration(self, component_name, service_name, callable_method, callable_is_done):
        """ This method is meant to be overloaded by middlewares that have
        specific initializations to do when a new service is exposed.

        :param component_name: name of the component that declare this service
        :param service_name: Public name of the service
        :param callable callable_method: Method to invoke on incoming request
        :param callable_is_done: If defined, means that the service can last.
               In this case, callable_is_done returns True when the service has ter
               minated. See :py:meth:register_service for details.
        :type callable_is_done: callable or None
        :return True if the registration succeeded.
        :rtype boolean
        """
        pass


    def register_service(self, component_name, callable_method, callable_is_done = None, service_name = None):
        """ Allows a component to register a RPC method that is made
        publicly available to the outside world.

        :param string component_name: name of the component that declare this service
        :param callable_method: the method name to invoke on incoming
               request.
               If service_name is not defined, it will also be used as
               the public name of the service.
               If callable_is_done is not defined, the method is expected to
               return immediately. In this case, its return value is send
               back to the original caller.
        :param callable_is_done: a method that returns a tuple (True,
               return_value) if the callback method has terminated,
               (False, None) else. This allows services to last for
               several cycles without blocking the communication interface.
        :param service_name: if defined, service_name is used as public
               name for this RPC method.
        """
        
        if callable(callable_method):
            service_name = service_name if service_name else callable_method.__name__

            name = component_name + "#" + service_name
            #Check that, if callable_is_done is provided, it is actually callable
            if callable_is_done and not callable(callable_is_done):
                print(str(self) + ": ERROR while registering a new service: the provided 'is_done' callable is not callable!")
                return

            self._services[name] = (callable_method, callable_is_done)

            if self._post_registration(component_name, name, callable_method, callable_is_done):
                print(str(self) + ": New service " + name + " for " + component_name + " successfully registered")
            else:
                print(str(self) + ": ERROR while registering a new service: could not complete the post-registration step.")

        else:
            print (str(self) + ": ERROR while registering a new service: " + str(callable_method) + \
                    " is not a callable object.")

    def _on_incoming_request(self, component, service, params):
        """ This method handles incoming requests: it figures out who
        registered the service, checks if the service returns immediately
        or must be started and only later checked for termination, invokes
        the service, and returns the service result (for service that returns
        immediately).

        If something goes wrong while trying to call the method, a
        :py:class:MorseRPCInvokationError is raised.

        If everything goes well, the method return a tuple: (True, return_value) or
        (False, None). The first item tells if the method returns

        """

        print("Incoming request " + service + " for " + component + "!")
        try:
            method, is_done = self._services[component + "#" + service]
        except KeyError:
            raise MorseRPCInvokationError("The request " + service + " has not been registered in " + str(self))

        if is_done: #A callable has been defined to test when the service ends: its a 'long duration' service
            self._pending_calls[service] = is_done

            #Invoke the method with unpacked parameters
            try:
                ok = method(*params) if params else method()
            except TypeError as e:
                raise MorseRPCInvokationError(str(self) + ": ERROR: wrong number of parameters for service " + service + ". " + str(e))

            if ok:
                print("Long-term request -> successfully dispatched to the component.")
                return (False, None)
            else:
                raise MorseRPCInvokationError(str(self) + ": ERROR: RPC call " + service + " could not get dispatched.")

        else: #Short call.
            try:
                #Invoke the method
                print("Short-term request -> invoking it now.")
                try:
                    values = method(*params) if params else method()
                except TypeError as e:
                    raise MorseRPCInvokationError(str(self) + ": ERROR: wrong number of parameters for service " + service + ". " + str(e))

                print("Done. Result: " + str(values))
                return (True, values) #Invoke the method with unpacked parameters
            except:
                raise MorseRPCInvokationError(str(self) + ": ERROR: An exception occured during the execution of the service " + service + "!")

    def _update_pending_calls(self):
        """This method is called at each simulation steps and check if pending requests are
        completed or not.
        On completion, it calls the :py:meth:_is_completed method.
        """

        if self._pending_calls:
            for request, is_done in self._pending_calls.items():
                done, value = is_done()

            if done:
                print(str(self) + ": " + request + " is now completed.")
                del self._pending_calls[request]
                self._on_service_completion(component, request, value)

    @abstractmethod
    def _on_service_completion(self, component, request, result):
        """ This method is called when a 'long term' request completes.

        Subclasses are expected to overload this method with code to notify
        the original request emitter.

        TODO: For now, this method does not allow to know if the service call
        has been successful or not. This should be returned by the 'is_done' 
        callable. To be defined!
        """
        pass

    @abstractmethod
    def _main(self):
        """ This is the main method of the RequestManagerClass: it reads
        external incoming requests, dispatch them through the
        :py:meth:_on_incoming_request method, and write back answers.
        
        Subclasses are expected to overload this method.
        """
        pass

    def process(self):
        """This method is the one actually called from the MORSE main loop.

        It simply updates the list of pending requests (if any) and calls
        the main processing method.
        """
        self._update_pending_calls()
        self._main()
