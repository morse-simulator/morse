import logging; logger = logging.getLogger("morse." + __name__)
try:
    import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('rosmorse')
    import rospy
    from rosmorse.srv import *
except ImportError as ie:
    raise ImportError("Could not import the 'rosmorse' ROS node. Check" +\
            "your ROS configuration is ok. Details:\n" + str(ie))

from morse.core import status
from morse.core.request_manager import RequestManager

class RosRequestManager(RequestManager):

    def __init__(self):
        RequestManager.__init__(self)

        self.services = []
        self.actions = []

    def __str__(self):
        return "ROS Request Manager"

    def initialization(self):

        # Init MORSE-node in ROS
        rospy.init_node('morse', log_level = rospy.DEBUG)
        logger.info("ROS node 'morse' successfully created.")

        return True

    def finalization(self):
        return True

    def post_registration(self, component_name, service_name, is_async):
        """ We create here ROS services (for *synchronous* services) and ROS
        actions (for *asynchronous* services).

        In order not to interfere with neither ROS own service wrapping mechanism or
        MORSE service invokation mechanisms, we create here, 'on the fly', handlers 
        for each service exposed through ROS.
        """

        service_name = service_name.split("#")[-1]

        cb = self.add_ros_handler(component_name, service_name)
        rostype = self.retrieve_type(component_name, service_name)

        s = rospy.Service("morse/" + component_name + "/" + service_name, rostype, cb)
        logger.info("Created new ROS service for {}.{}".format(
                                                    component_name,
                                                    service_name))
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
                raise rospy.ServiceException(result)

        innermethod.__doc__ = "This method is invoked directly by MORSE when " +\
        "a new service request comes in. This handler simply redispatch it to " +\
        "MORSE own service invokation system.\n\n" +\
        "The method takes a ROS ServiceRequest as unique parameter, and must " +\
        "return a ROS ServiceResponse"

        innermethod.__name__ = "_" + service_name + "_ros_handler"

        #setattr(self, innermethod.__name__, innermethod)
        logger.debug("Created new ROS->MORSE dispatcher for service {}.{}".format(
                                                    component_name,
                                                    service_name))
        return innermethod

    def retrieve_type(self, component_name, service_name):
        # TODO: store a dictionary somewhere with service->type
        # matching?
        return ListRobots

    def on_service_completion(self, request_id, result):
        pass

    def main(self):
        pass
