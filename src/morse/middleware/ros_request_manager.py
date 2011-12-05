import logging; logger = logging.getLogger("morse." + __name__)

from functools import partial

from morse.core import status, services
from morse.core.request_manager import RequestManager

try:
    import roslib; roslib.load_manifest('rospy')
    import rospy
except ImportError as ie:
    raise ImportError("Could not import 'rospy' ROS module."
                      " Check your ROS configuration is ok. Details:\n" + str(ie))

class RosRequestManager(RequestManager):

    def __init__(self):
        RequestManager.__init__(self)

        self.services = []
        self.actions = []

    def __str__(self):
        return "ROS Request Manager"

    def initialization(self):

        # Init MORSE-node in ROS
        rospy.init_node('morse', log_level = rospy.DEBUG, disable_signals=True)
        logger.info("ROS node 'morse' successfully created.")

        return True

    def finalization(self):
        logger.info("Shutting down ROS node...")
        rospy.signal_shutdown("User exited MORSE simulation")
        return True

    def register_ros_action(self, method, component_name, service_name):
        
        # Default service type
        #rostype = rospy.msg.AnyMsg
        
        #try:
        #    rostype = method._ros_action_type # Is it a ROS action?
        #    logger.info(component_name + "." + service_name + " is a ROS action of type " + str(rostype))
        #except AttributeError:
        #    logger.info(component_name + "." + service_name + " has no ROS-specific action type. Using default one.")
        #
        #cb = self.add_ros_handler(component_name, service_name)
        #
        #s = actionlib.SimpleActionServer(service_name, rostype, cb)
        #
        #logger.info("Created new ROS action server for {}.{}".format(
        #                                            component_name,
        #                                            service_name))
        pass


    def register_ros_service(self, method, component_name, service_name):
        
        # Default service type
        rostype = MorseAnyService
        
        try:
            rostype = method._ros_service_type # Is it a ROS service?
            logger.debug(component_name + "." + service_name + " is a ROS service of type " + str(rostype))
        except AttributeError:
            logger.debug(component_name + "." + service_name + " has no ROS-specific service type. Using default one.")
        
        cb = self.add_ros_handler(component_name, service_name)

        s = rospy.Service(service_name, rostype, cb)
        logger.debug("Created new ROS service for {}.{}".format(
                                                    component_name,
                                                    service_name))
    
    def post_registration(self, component_name, service_name, is_async):
        """ We create here ROS services (for *synchronous* services) and ROS
        actions (for *asynchronous* services).

        In order not to interfere with neither ROS own service wrapping mechanism or
        MORSE service invokation mechanisms, we create here, 'on the fly', handlers 
        for each service exposed through ROS.
        
        ROS requires type for the services/actions. Those can be set with the
        :py:method:`ros_action` and :py:method:`ros_service` decorators.
        If none is set, a default type is used (:py:class:`MorseAnyService`).
        """
        
        rostype = None
        
        # TODO: I access here an internal member of the parent class to 
        # retrieve the ROS type, if set. _services should probably be 
        # 'officially' exposed
        method, is_async = self._services[(component_name, service_name)]
        
        if is_async:
            self.register_ros_action(method, component_name, service_name)
        else:
            self.register_ros_service(method, component_name, service_name)



        return True

    def add_ros_handler(self, component_name, service_name):
        """ Dynamically creates custom ROS->MORSE dispatchers.
        """

        def innermethod(request):
            # TODO: when this will be actually called? If ROS service
            # management lives in its own thread, it can be basically
            # called at any time, which is dangerous.

            # The request type is generated from the .srv definition.
            # roslib.message.Message defines the method __getstate__()
            # the retrieve a 'flat' view of all parameters.
            args = request.__getstate__()

            logger.info("ROS->MORSE dispatcher for " + service_name + \
                         " got incoming request: " + str(args))
            is_sync, value = self.on_incoming_request(component_name, service_name, args)

            # is_sync should be always True for ROS services!

            state, result = value

            if state == status.SUCCESS:
                # Here, we 'hope' that the return value of the MORSE
                # service is a valid dataset to construct the
                # ServiceResponse expected by ROS
                return (result, )
            else:
                # failure!
                raise rospy.service.ServiceException(result)

        innermethod.__doc__ = "This method is invoked directly by MORSE when " +\
        "a new service request comes in. This handler simply redispatch it to " +\
        "MORSE own service invokation system.\n\n" +\
        "The method takes a ROS ServiceRequest as unique parameter, and must " +\
        "return a ROS ServiceResponse"

        innermethod.__name__ = "_" + service_name + "_ros_handler"
        
        logger.debug("Created new ROS->MORSE dispatcher for service {}.{}".format(
                                                    component_name,
                                                    service_name))
        return innermethod

    def on_service_completion(self, request_id, result):
        pass

    def main(self):
        pass

class MorseAnyService(roslib.message.ServiceDefinition):
    _type = 'morse/AnonymousService'
    _md5sum = ''
    _request_class = rospy.msg.AnyMsg
    _response_class = rospy.msg.AnyMsg


def ros_action(fn = None, type = None, name = None):
    """ The @ros_action decorator.
    
    This decorator is very similar to the standard 
    :py:meth:`morse.core.services.async_service` decorator. It sets a 
    class method to be a asynchronous service, exposed as a ROS action of
    type `type`.
    
    This decorator can only be used on methods in classes inheriting from 
    :py:class:`morse.core.object.MorseObjectClass`.
    
    :param callable fn: [automatically set by Python to point to the
      decorated function] 
    :param class type: you MUST set this parameter to define the
      type of the ROS action.
    :param string name: by default, the name of the service is the name
      of the method. You can override it by setting the 'name' argument.
      Your ROS action will appear as /<name>/...
    """
    if not type:
        logger.error("You must provide a valid ROS action type when using the " + \
        "@ros_action decorator, e.g. @ros_action(type=MyRosAction). Action ignored.")
        return
        
    if not hasattr(fn, "__call__"):
        return partial(ros_action, type = type, name = name)
        
    fn._ros_action_type = type
    return services.service(fn, component = None, name = name, async = True)
        

def ros_service(fn = None, type = None, component = None, name = None):
    """ The @ros_service decorator.
    
    This decorator is very similar to the standard 
    :py:meth:`morse.core.services.service` decorator. It sets a free function or
    class method to be a (synchronous) service, exposed as a ROS service of
    type `type`.

    This decorator works both with free function and for methods in
    classes inheriting from
    :py:class:`morse.core.object.MorseObjectClass`. In the former case,
    you must specify a component (your service will belong to this
    namespace), in the later case, it is automatically set to the name
    of the corresponding MORSE component.

    :param callable fn: [automatically set by Python to point to the
      decorated function] 
    :param class type: you MUST set this parameter to define the
      type of the ROS action.
    :param string component: you MUST set this parameter to define the
      name of the component which export the service ONLY for free
      functions. Cf explanation above.
    :param string name: by default, the name of the service is the name
      of the method. You can override it by setting the 'name' argument.
      Your ROS service will appear as /<name>
    """    
    if not type:
        logger.error("You must provide a valid ROS action type when using the " + \
        "@ros_action decorator, e.g. @ros_action(type=MyRosAction). Service ignored.")
        return
        
    if not hasattr(fn, "__call__"):
        return partial(ros_service, type = type, component = component, name = name)

    fn._ros_service_type = type
    return services.service(fn, component = component, name = name, async = False)
