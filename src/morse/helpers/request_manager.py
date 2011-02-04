import os
import sys
import uuid
from functools import partial

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
        - :py:meth:_initialization: perform here middleware specific initialization
        - :py:meth:_finalization: perform here middleware specific finalization
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
        # (rpc_callback, is_async)
        self._services = {}

        # This hold the mapping request id <-> result for asynchronous
        # requests.
        # Keys are request ids, values are either 'None' for pending
        # requests or a tuple (True|False, result|error_msg) for
        # completed service calls.
        # It is updated on each call to :py:meth:_update_pending_calls()
        self._completed_requests = {}


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
    def _post_registration(self, component_name, service_name, callback, is_async):
        """ This method is meant to be overloaded by middlewares that have
        specific initializations to do when a new service is exposed.

        :param string component_name: name of the component that declare this service
        :param string service_name: Public name of the service
        :param callable callback: Method to invoke on incoming request
        :param boolean is_async: If true, means that the service is asynchronous.
        :return True if the registration succeeded.
        :rtype boolean
        """
        pass


    def register_async_service(self, component_name, callback, service_name = None):
        """ Allows a component to register an asynchronous RPC method.

        A asynchronous method can last for several cycles without blocking the simulator.
        The callback method must take as first parameter a callable that must be used
        to set the results of the service upon completion.

        For example, consider the following sample of asynchronous service:

            def complex_computation(result_setter, param1, param2):
                do_computation_step() #should stay short, but can last several simulation steps

                if computation_done:
                    result_setter(computation_results)

            request_manager.register_async_service("computer", complex_computation)

        As soon as the 'result_setter' is called with the results of the service,
        the clients of this service are notified via their middlewares.

        See :py:meth:register_service for detailed documentation of parameters.
        """
        self.register_service(component_name, callback, service_name, True)


    def register_service(self, component_name, callback, service_name = None, async = False):
        """ Allows a component to register a synchronous RPC method that is made
        publicly available to the outside world.

        :param string component_name: name of the component that declare this service
        :param callable callback: the method name to invoke on incoming
               request.
               If service_name is not defined, it will also be used as
               the public name of the service.
               If async is false (synchronous service), the method is expected to
               return immediately. In this case, its return value is immediately
               send back to the original caller.
        :param boolean async: if true, the service is asynchronous: it can last for
               several cycles without blocking the communication interface.
               See :py:meth:register_async_service for details.
        :param service_name: if defined, service_name is used as public
               name for this RPC method.
        """
        
        if callable(callback):
            service_name = service_name if service_name else callback.__name__

            name = component_name + "#" + service_name

            self._services[name] = (callback, async)

            if self._post_registration(component_name, name, callback, async):
                print(str(self) + ": New " + \
                    ("asynchronous" if async else "synchronous") + " service " + \
                    name + " for " + component_name + " successfully registered")
            else:
                print(str(self) + ": ERROR while registering a new service: " + \
                        "could not complete the post-registration step.")

        else:
            print (str(self) + ": ERROR while registering a new service: " + callback.__name__ + \
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
        (False, request_id). The first item tells if the service is a synchronous
        (short-term) service (value is True) or an asynchronous service (False).

        For asynchronous services, the returned request id should allow to track
        the completion of the service. Upon completion, :py:meth:_on_completion
        is invoked.

        """

        print("Incoming request " + service + " for " + component + "!")

        request_id = uuid.uuid4() #Unique ID for our request
        
        try:
            method, is_async = self._services[component + "#" + service]
        except KeyError:
            raise MorseRPCInvokationError("The request " + service + " has not been registered in " + str(self))

        if is_async:
            # Creates a result setter functor
            self._completed_requests[request_id] = None
            result_setter = partial(self._completed_requests.__setitem__, request_id)
            #Invoke the method with unpacked parameters
            try:
                ok = method(result_setter, *params) if params else method(result_setter)
            except TypeError as e:
                raise MorseRPCInvokationError(str(self) + ": ERROR: wrong parameters for service " + service + ". " + str(e))

            if ok:
                print("Asynchronous request -> successfully started.")
                return (False, request_id)
            else:
                raise MorseRPCInvokationError(str(self) + ": ERROR: RPC call " + service + " could not get dispatched.")

        else: #Synchronous service.
            try:
                #Invoke the method
                print("Sychronous service -> invoking it now.")
                try:
                    values = method(*params) if params else method() #Invoke the method with unpacked parameters
                except TypeError as e:
                    raise MorseRPCInvokationError(str(self) + ": ERROR: wrong parameters for service " + service + ". " + str(e))

                print("Done. Result: " + str(values))
                return (True, values)
            except:
                raise MorseRPCInvokationError(str(self) + ": ERROR: An exception occured during the execution of the service " + service + "!")

    def _update_pending_calls(self):
        """This method is called at each simulation steps and check if pending requests are
        completed or not.
        On completion, it calls the :py:meth:_is_completed method.
        """

        if self._completed_requests:
            for request, result in self._completed_requests.items():
                if result:
                    print(str(self) + ": Request " + str(request) + " is now completed.")
                    del self._completed_requests[request]
                    self._on_service_completion(request, result)

    @abstractmethod
    def _on_service_completion(self, request_id, result):
        """ This method is called when a asynchronous request completes.

        Subclasses are expected to overload this method with code to notify
        the original request emitter.

        :param uuid request_id: the request id, as return by :py:meth:_on_incoming_request
                    when processing an asynchronous request
        :param tuple result: a tuple (True|False, result|error_msg) depending
                    on the execution result.
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
