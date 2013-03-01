import logging; logger = logging.getLogger("morse." + __name__)
import sys


from functools import partial

from morse.core import blenderapi
from morse.core.exceptions import MorseServiceError
from morse.helpers.loading import create_instance

class MorseServices:
    def __init__(self, impls = []):
        """ Initializes the different MORSE request managers from a list
        of Python classes.

        :param list impls: a list of Python class names (strings) with their module
                path that implement the RequestManager interface.
        """
        self._request_managers = {}
        self._service_mappings = {}
    
        for impl in impls:
            self.add_request_manager(impl)

    def add_request_manager(self, classpath):
        """ Initializes and adds a new request manager from its name.

        :param string classpath: the name (and path) of the Python class that
                implements the RequestManager interface (eg:
                'morse.middleware.socket_request_manager.SocketRequestManager',
                'morse.middleware.yarp_request_manager.YarpRequestManager',...).
        :return: True if the request manager has been successfully loaded.
        """
        # Check if the request manager do not already exist
        if not classpath in self._request_managers:
            instance = create_instance(classpath)
            if not instance:
                logger.error("Request Manager %s not found. Check for typos in the configuration file!"%classpath)
                return False

            # In case of instantiation failure, this may raise a MorseServiceError exception
            self._request_managers[classpath] = instance
            logger.info("Successfully initialized the %s request manager." % classpath)
        
        return True


    def register_request_manager_mapping(self, component, request_manager):
        """Adds a mapping between a component and a request manager: all
        services exposed by this component are handled by this request manager.

        A component may have 0, 1 or more request managers. if more than one,
        each request manager can independently invoke the service.
        
        :param string component: the name of the component that use *request_manager*
                as request manager.
        :param string request_manager: the classpath of the request manager (eg:
                'morse.middleware.socket_request_manager.SocketRequestManager',
                'morse.middleware.yarp_request_manager.YarpRequestManager',...).
        """

        if not request_manager in self._request_managers:
            raise MorseServiceError("Request manager '%s' has not been registered!" % request_manager)

        instance = self._request_managers[request_manager]

        if component in self._service_mappings:
            self._service_mappings[component].add(instance)
        else:
            self._service_mappings[component] = {instance, }


    def __del__(self):
        """ Removes all registered request managers, calling their destructors. """
        logger.info("Deleting all request managers...")
        for rqst_manager in self._request_managers.values():
            if not rqst_manager.finalization():
                logger.warning("finalization of the service manager did not complete successfully!")
            
            logger.info("%s: service manager closed." % rqst_manager)
            
        self._request_managers.clear()
        self._service_mappings.clear()
        
    def get_request_managers(self, component):
        if not component in self._service_mappings:
            logger.error("no service manager is available for the " + component + " component! This error " +  \
                "can have several causes. Maybe you forgot to add the middleware 'empty', or " + \
                "you are using a middleware that does not currently support requests. ")
            raise MorseServiceError("No request manager has been mapped to the component %s" % component)
        
        return self._service_mappings[component]

    def request_managers(self):
        """ Returns the list of active request managers.

        :returns: a dictionary of active request managers {class name: instance}
        """
        return self._request_managers

    def process(self):
        """ Calls the *process()* method of each registered request manager.
        """
        for name, instance in self._request_managers.items():
            instance.process()


def do_service_registration(fn, component_name = None, service_name = None, async = False, request_managers = None):

    if blenderapi.fake: #doc mode
        return

    if not component_name:
        logger.error("A service has been registered without component: " + str(fn))
        return

    if not request_managers:
        request_managers = blenderapi.persistantstorage().morse_services.get_request_managers(component_name)

    for manager in request_managers:
        name = service_name if service_name else fn.__name__
        logger.debug("Registering service " + name + " in " + component_name + " (using " + manager.__class__.__name__ + ")")
        manager.register_service(component_name, fn, name, async)

def async_service(fn = None, component = None, name = None):
    """  The @async_service decorator.

    Refer to the @service decorator for most of the doc.

    Asynchronous service specifics:

    - The function that is decorated is expected to simply start the
      service, and immediately return.
    - If the service can not be started, the function must throw a
      :py:class:`MorseRPCInvokationError` with a error message
      explaining why the initialization failed.  

      """
    return service(fn, component, name, async = True)

def service(fn = None, component = None, name = None, async = False):
    """ The @service decorator.

    This decorator can be used to automagically register a service in
    MORSE. Simply decorate the method you want to export as a RPC service
    with @service and MORSE automatically add and register it with the
    right middleware (depending on what is specified in the simulation
    configuration file).

    This decorator works both with free function and for methods in
    classes inheriting from
    :py:class:`morse.core.object.Object`. In the former case,
    you must specify a component (your service will belong to this
    namespace), in the later case, it is automatically set to the name
    of the corresponding MORSE component.

    :param callable fn: [automatically set by Python to point to the
      decorated function] 
    :param string component: you MUST set this parameter to define the
      name of the component which export the service ONLY for free
      functions. Cf explanation above.
    :param string name: by default, the name of the service is the name
      of the method. You can override it by setting the 'name' argument.
    :param boolean async: if set to True (default value when using 
      @async_service), a new 'callback' parameter is added to the method.
      This callback is used to notify the service initiator that the service
      completed. The callback does not need to be build manually: 
      :py:meth:`morse.core.request_manager.RequestManager.on_incoming_request`
      takes care of it.
    """
    if hasattr(fn, "__call__"):
        # If the @service decorator has no explicit parameter, then Python
        # pass directly the function -> a callable. We can register it.
        if not component:
            # If component is not defined, we assume it is a class method.
            # In this case, the service registration is defered to the
            # class instanciation (cf object.py), and we simply mark
            # this method as a service.
            logger.debug("In @service: Decorating method "+ fn.__name__)
            dfn = fn
            if async:
                def decorated_fn(self, callback, *param):
                    # Stores in the callback the original calling
                    # service.
                    try:
                        callback.service = decorated_fn
                    except AttributeError:
                        raise MorseServiceError("Invalid callback for async service. Did you forget to pass the chain callback from an overlay?")

                    self.set_service_callback(callback)
                    try:
                        fn(self, *param)
                    except BaseException as e:
                        # Failure during service invokation? remove the
                        # callback and re-raise
                        self.on_completion = None
                        raise e

                dfn = decorated_fn
                dfn.__name__ = fn.__name__
                dfn.__doc__ = fn.__doc__

                # Copy all special values the original method may have.
                # This is useful in case of cascading decorator (cf
                # @ros_action for instance).
                for attr, value in fn.__dict__.items():
                    setattr(dfn, attr, value)

            dfn._morse_service = True
            dfn._morse_service_name = name
            dfn._morse_service_is_async = async

            return dfn

        else:
            if async:
                logger.warning("asynchronous service must be declared within a MorseObject class.")
                return

            logger.debug("In @service: Decorating free function "+ fn.__name__)
            # We assume it's a free function, and we register it.
            do_service_registration(fn, component, name, async)
            return fn
    else:
         # ...else, we build a new decorator
        return partial(service, component = component, name = name, async = async)

def interruptible(fn):
    """ The @interruptible decorator.

    Use this decorator to set an (asynchronous) service to be 
    interruptible.

    If MORSE receives a request for a new service while an
    interruptible service is running, the running service is
    preempted (the requester receives a :data:`morse.core.status.PREEMPTED` 
    status), and the new one is started.

    See also :meth:`noninterruptible` decorator.
    """
    logger.debug("In @interruptible: Decorating method "+ fn.__name__)
    fn._morse_service_interruptible = True

    return fn

def noninterruptible(fn):
    """ The @noninterruptible decorator.

    Use this decorator to set an (asynchronous) service to be non
    interruptible.

    If MORSE receives a request for a new service while a non
    interruptible service is running, a failure message is returned
    to the requester.

    See also :meth:`interruptible` decorator.
    """
    logger.debug("In @noninterruptible: Decorating method "+ fn.__name__)
    fn._morse_service_interruptible = False

    return fn

