import logging; logger = logging.getLogger("morse." + __name__)
#logger.setLevel(logging.DEBUG)
import os
import sys
import uuid
from functools import partial
from abc import ABCMeta, abstractmethod

from morse.core.exceptions import *
from morse.core import status, blenderapi

class RequestManager(object):
    """ Basic Class for all request dispatchers, i.e., classes that
    implement a *request service*.

    A request service offers typically 2 things:
      - the ability for a component (a robot, a sensor or the
        simulator as a whole) to expose a RPC method (typically for
        remote configuration or debug),
      - an interface with a specific middleware to serialize the 
        RPC call and communicate with the outside world.

    Components can register such a service with the 'register_service'
    method. Please check its documentation for details.
 
    To implement a concrete RequestManager (for a new middleware, for instance),
    the following methods must be overloaded:
        - :py:meth:`initialization`: perform here middleware specific initialization
        - :py:meth:`finalization`: perform here middleware specific finalization
        - :py:meth:`post_registration`: put here all middleware specific code
          that must be executed when a new service is registered.
        - :py:meth:`on_service_completion`: this method is called when a 'long term'
          request completes. You should implement here a way to notify
          your clients.
        - :py:meth:`main`: this method is called at each step of the
          simulation. You should read there incoming requests and write back
          results.

    When a new request arrives, you must pass it to :py:meth:`on_incoming_request`
    that dispatch or invoke properly the request.

    Subclasses are also expected to overload the special :py:meth:`__str__`
    method to provide middleware specific names.

   """

    # Make this an abstract class
    __metaclass__ = ABCMeta

    def __init__ (self):
        """ Constructor method.
        """

        # This map holds the list of all registered services
        # It associates a tuple (component,service) to a tuple
        # (rpc_callback, is_async)
        self._services = {}

        # This hold the mapping request id <-> result for asynchronous
        # requests.
        # Keys are request ids, values are either 'None' for pending
        # requests or a tuple (True|False, result|error_msg) for
        # completed service calls.
        # It is updated on each call to :py:meth:`_update_pending_calls`
        self._completed_requests = {}

        # Holds a mapping request_id -> (component, service)
        self._pending_requests = {}


        if not self.initialization():
            raise MorseServiceError("Couldn't create the service manager! Initialization failure")

    @abstractmethod
    def initialization(self):
        """This method is meant to be overloaded by middlewares to perform
        specific initializations.

        Must return True is the initialization is successful, False in other cases.
        """
        pass

    @abstractmethod
    def finalization(self):
        """This method is meant to be overloaded by middlewares to perform
        specific finalizations.

        Must return True is the finalization is successful, False in other cases.
        """
        pass

    def __str__(self):
        """ Should be overloaded by subclasses to help debug request handling
        for each middleware implementation.
        """
        return "Generic request manager"

    @abstractmethod
    def post_registration(self, component_name, service_name, is_async):
        """ This method is meant to be overloaded by middlewares that have
        specific initializations to do when a new service is exposed.

        :param string component_name: name of the component that declare this 
            service
        :param string service_name: Name of the service (if not overloaded 
            in the @service decorator, should be the Python function name that
            implement the service)
        :param boolean is_async: If true, means that the service is asynchronous.
        :return: True if the registration succeeded.
        :rtype: boolean
        """
        pass


    def register_async_service(self, component_name, callback, service_name = None):
        """ Allows a component to register an asynchronous RPC method.

        A asynchronous method can last for several cycles without blocking the simulator.
        The callback method must take as first parameter a callable that must be used
        to set the results of the service upon completion.

        For example, consider the following sample of asynchronous service::

            def complex_computation(result_setter, param1, param2):
                do_computation_step() #should stay short, but can last several simulation steps

                if computation_done:
                    result_setter(computation_results)

            request_manager.register_async_service("computer", complex_computation)

        As soon as the 'result_setter' is called with the results of the service,
        the clients of this service are notified via their middlewares.

        See :py:meth:`register_service` for detailed documentation of parameters.
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
               See :py:meth:`register_async_service` for details.
        :param service_name: if defined, service_name is used as public
               name for this RPC method.
        """
        
        if hasattr(callback, '__call__'):
            service_name = service_name if service_name else callback.__name__

            self._services[(component_name, service_name)] = (callback, async)

            if self.post_registration(component_name, service_name, async):
                logger.info(str(self) + ": " + \
                    ("Asynchronous" if async else "Synchronous") + \
                    " service '" + service_name + "' for component '" + \
                    component_name + "' successfully registered")
            else:
                logger.info(str(self) + ": Did not register service <%s> " % service_name + \
                        "(could not complete the post-registration step).")

        else:
            logger.error(str(self) + ": Error while registering a new service: " + str(callback) + \
                    " is not a callable object.")
    
    def services(self):
        """ Returns the list of all components and services registered with this
        request manager.
        
        :return: a dictionary of {components:[services...]} listing all services
                 registered with this request manager.
        """
        services = {}
        for component, service in self._services.keys():
            services.setdefault(component, []).append(service)
        
        return services
        
    def on_incoming_request(self, component, service, params):
        """ This method handles incoming requests: it figures out who
        registered the service, checks if the service returns immediately
        or must be started and only later checked for termination, invokes
        the service, and returns the service result (for service that returns
        immediately).

        If something goes wrong while trying to call the method, a
        :py:class:`morse.core.exceptions.MorseRPCInvokationError` is raised.

        If everything goes well, the method return a tuple: ``(True,
        return_value)`` or ``(False, request_id)``. The first item tells
        if the service is a synchronous (short-term) service (value is
        ``True``) or an asynchronous service (``False``).

        For asynchronous services, the returned request id should allow to track
        the completion of the service. Upon completion, :py:meth:`on_service_completion`
        is invoked.

        """

        logger.info("Incoming request " + service + " for " + component + "!")

        #Unique ID for our request
        request_id = uuid.uuid1()

        try:
            method, is_async = self._services[(component, service)]
        except KeyError:
            raise MorseMethodNotFoundError("The request " + service + " has not been registered in " + str(self))

        if is_async:

            # Creates a result setter functor: this functor is used as
            # callback for the asynchronous service.
            result_setter = partial(self._completed_requests.__setitem__, request_id)
            try:
                # Invoke the method with unpacked parameters
                # This method may throw MorseRPCInvokationError if the
                # service initialization fails.
                method(result_setter, *params) if params else method(result_setter)

                # Store the component and service associated to this service
                # (for instance, for later interruption)
                self._pending_requests[request_id] = (component, service)

            except AttributeError as e:
                raise MorseRPCTypeError(str(self) + ": wrong parameter type for service " + service + ". " + str(e))
            except TypeError as e:

                # Check if the type error comes from a wrong # of args.
                # We perform this check only after an exception is
                # thrown to avoid loading the inspect module by default.
                import inspect, traceback
                logger.debug(traceback.format_exc())
                if not params:
                    raise MorseRPCNbArgsError(str(self) + ": parameters expected for service " + service + "! " + str(e))
                elif len(params) != (len(inspect.getargspec(method)[0]) - 2): # -2 because of self and callback
                    raise MorseRPCNbArgsError(str(self) + ": wrong # of parameters for service " + service + ". " + str(e))
                else:
                    raise MorseRPCTypeError(str(self) + ": wrong parameter type for service " + service + ". " + str(e))

            logger.debug("Asynchronous request '" + str(request_id) + "' successfully started.")
            return (False, request_id)

        else: #Synchronous service.
            #Invoke the method
            logger.info("Synchronous service -> invoking it now.")
            try:
                values = method(*params) if params else method() #Invoke the method with unpacked parameters
            except AttributeError as e:
                raise MorseRPCTypeError(str(self) + ": wrong parameter type for service " + service + ". " + str(e))
            except TypeError as e:
                # Check if the type error comes from a wrong # of args.
                # We perform this check only after an exception is
                # thrown to avoid loading the inspect module by default.
                # TODO: Does it make sense?
                import inspect, traceback
                logger.debug(traceback.format_exc())
                if not params:
                    raise MorseRPCNbArgsError(str(self) + ": parameters expected for service " + service + "! " + str(e))
                if len(params) != (len(inspect.getargspec(method)[0]) - 1): # -1 because of 'self'
                    raise MorseRPCNbArgsError(str(self) + ": wrong # of parameters for service " + service + ". " + str(e))
                else:
                    raise MorseRPCTypeError(str(self) + ": wrong parameter type for service " + service + ". " + str(e))

            # If we are here, no exception has been raised by the
            # service, which mean the service call is successful. Good.
            values = (status.SUCCESS, values)
            logger.info("Done. Result: " + str(values))
            return (True, values)

    def abort_request(self, request_id):
        """ This method will interrupt a running asynchronous service,
        uniquely described by its request_id
        """
        component_name, service_name = self._pending_requests[request_id]

        for component in blenderapi.persistantstorage().componentDict.values():
            if component.name() == component_name:
                logger.info("calling  interrupt on %s" % str(component))
                component.interrupt()
                return

        # if not found, search in the overlay dictionnary
        for overlay in blenderapi.persistantstorage().overlayDict.values():
            if overlay.name() == component_name:
                logger.info("calling  interrupt on %s" % str(overlay))
                overlay.interrupt()
                return


    def _update_pending_calls(self):
        """This method is called at each simulation steps and check if pending requests are
        completed or not.
        On completion, it calls the :py:meth:`on_service_completion` method.
        """

        if self._completed_requests:
            for request, result in list(self._completed_requests.items()):
                if result:
                    logger.debug(str(self) + ": Request " + str(request) + " is now completed.")
                    del self._pending_requests[request]
                    del self._completed_requests[request]
                    self.on_service_completion(request, result)

    @abstractmethod
    def on_service_completion(self, request_id, result):
        """ This method is called when a asynchronous request completes.

        Subclasses are expected to overload this method with code to notify
        the original request emitter.

        :param uuid request_id: the request id, as return by :py:meth:`on_incoming_request`
                    when processing an asynchronous request
        :param result: the service execution result.
        """
        pass

    @abstractmethod
    def main(self):
        """ This is the main method of the RequestManagerClass: it reads
        external incoming requests, dispatch them through the
        :py:meth:`on_incoming_request` method, and write back answers.
        
        Subclasses are expected to overload this method.
        """
        pass

    def process(self):
        """This method is the one actually called from the MORSE main loop.

        It simply updates the list of pending requests (if any) and calls
        the main processing method.
        """
        self._update_pending_calls()
        self.main()
