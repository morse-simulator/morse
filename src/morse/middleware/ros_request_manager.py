import logging; logger = logging.getLogger("morse." + __name__)

import threading
from functools import partial

from morse.core import status, services
from morse.core.request_manager import RequestManager

try:
    import roslib; roslib.load_manifest('rospy'); roslib.load_manifest('actionlib_msgs')
    import rospy
    import actionlib_msgs
except ImportError as ie:
    raise ImportError("Could not import some ROS modules."
                      " Check your ROS configuration is ok. Details:\n" + str(ie))

def ros_timer(callable,frequency):
    # Shamelessly stolen from actionlib/action_server.py

    rate = rospy.Rate(frequency)

    rospy.logdebug("Starting timer");
    while not rospy.is_shutdown():
        try:
            rate.sleep()
            callable()
        except rospy.exceptions.ROSInterruptException:
            rospy.logdebug("Sleep interrupted");

class RosAction:
    """ Implements a minimal action state machine.

    See http://www.ros.org/wiki/actionlib/DetailedDescription
    for the possible states.
    """
    def __init__(self, manager, component, action, rostype):

        self.manager = manager

        self._pending_goals = {}
        self.goal_lock = threading.Lock()

        self.component = component
        self.action = action

        self.name = component + "/" + action

        # Retrieves the types of the goal msg, feedback msg and result
        # msg (check roslib.message.Message for details)
        rosinstance = rostype()
        types = [type(getattr(rosinstance, t)) for t in rosinstance.__slots__]

        self.result_type = types[1]
        self.feedback_type = types[2]

        # Create the 5 topics required by an action server
        self.goal_topic = rospy.Subscriber(self.name + "/goal", types[0], self.on_goal)
        self.result_topic = rospy.Publisher(self.name + "/result", types[1])
        self.feedback_topic = rospy.Publisher(self.name + "/feedback", types[2])

        self.cancel_topic = rospy.Subscriber(self.name + "/cancel", actionlib_msgs.msg.GoalID, self.on_cancel)
        self.status_topic = rospy.Publisher(self.name + "/status", actionlib_msgs.msg.GoalStatusArray)

        # read the frequency with which to publish status from the parameter server
        # (taken from actionlib/action_server.py)
        self.status_frequency = rospy.get_param(self.name + "/status_frequency", 5.0);

        status_list_timeout = rospy.get_param(self.name + "/status_list_timeout", 5.0);
        self.status_list_timeout = rospy.Duration(status_list_timeout);

        self.status_timer = threading.Thread(None, ros_timer, None, (self._publish_status,self.status_frequency) );
        self.status_timer.start();

    def setstatus(self, id, status):

        with self.goal_lock:
            goal_id = self._pending_goals[id]['goal_id']
            goal_status = actionlib_msgs.msg.GoalStatus(goal_id = goal_id,
                                                   status = status)
            self._pending_goals[id]['status'] = goal_status

    def getstatus(self, id):

        with self.goal_lock:
            return self._pending_goals[id]['status'].status

    def set_internal_id(self, id, morse_id):

        with self.goal_lock:
            self._pending_goals[id]['morse_id'] = morse_id

    def manage_internal_id(self, morse_id):
        """ Check if this ROS action manager manages the given
        internal request ID.

        This is used by RosRequestManager to dispatch the service
        'on completion' event.
        """
        return self.get_id_from_internal_id(morse_id) != None

    def get_id_from_internal_id(self, morse_id):

        with self.goal_lock:
            for id, goal in self._pending_goals.items():
                if goal.setdefault('morse_id') == morse_id:
                    return id

    def on_goal(self, goal):
        logger.info("Got a new goal for ROS action " + self.name)

        # Workaround for encoding issues (-> goal_id.id comes as bytes,
        # and must be converted to string before being reserialized)
        id = actionlib_msgs.msg.GoalID(id = goal.goal_id.id,
                                       stamp = goal.goal_id.stamp)

        with self.goal_lock:
            self._pending_goals[id.id] = {'goal_id': id, 'status': None}

        self.setstatus(id.id, actionlib_msgs.msg.GoalStatus.PENDING)

        is_sync, morse_id = self.manager.on_incoming_request(self.component, self.action, [goal.goal])

        # is_sync should be always True for ROS services!
        if is_sync:
            # TODO: clean terminated goals 'after a few seconds' (as
            # said by actionlib doc)
            self.setstatus(id.id, actionlib_msgs.msg.GoalStatus.REJECTED)
            logger.error("Internal error: This ROS action is bound to a "
                         "synchronous MORSE service! ({})".format(component_name + '.' + service_name))

        self.set_internal_id(id.id, morse_id)
        self.setstatus(id.id, actionlib_msgs.msg.GoalStatus.ACTIVE)
        logger.debug("Started action. GoalID=" + id.id + " MORSE ID=" + str(morse_id))

    def on_cancel(self, goal_id):
        logger.info("Got a cancel request for ROS action " + self.name)

        current_status = None

        try:
            current_status = self.getstatus(goal_id.id)
        except KeyError:
            # Unknown GoalID...? skipping this cancel request.
            logger.info("I can not find any goal matching this cancel request. Skipping it.")
            return

        if current_status == actionlib_msgs.msg.GoalStatus.PENDING:
            self.setstatus(id.id, actionlib_msgs.msg.GoalStatus.RECALLING)
        else: #current status = ACTIVE (or smth else...)
            self.setstatus(id.id, actionlib_msgs.msg.GoalStatus.PREEMPTING)

    def on_result(self, morse_id, state, result):

        logger.info("Got a result for action " + self.name + ": " + str(state) + " " + str(result))
        id = self.get_id_from_internal_id(morse_id)

        if state == status.PREEMPTED:
            logger.info("The action " + self.name + " has been preempted. "
                         "Reporting it to ROS system.")
            self.setstatus(id, actionlib_msgs.msg.GoalStatus.PREEMPTED)
        if state == status.FAILED:
            logger.info("The action " + self.name + " has been aborted. "
                         "Reporting it to ROS system.")
            self.setstatus(id, actionlib_msgs.msg.GoalStatus.ABORTED)
        if state == status.SUCCESS:
            logger.debug("The action " + self.name + " has succeeded. Reporting"
                         " it to ROS system and publishing results.")
            self.setstatus(id, actionlib_msgs.msg.GoalStatus.SUCCEEDED)
            self.publish_result(id, result)

    def publish_result(self, goal_id, result):

        goal_status = None

        with self.goal_lock:
            goal_status = self._pending_goals[goal_id]['status']

        res = self.result_type(status = goal_status,
                               result = result)
        
        self.result_topic.publish(res)


    def _publish_status(self):
        """ This private method is called asynchronously to update the status of pending goals.
        """

        status_array = actionlib_msgs.msg.GoalStatusArray()

        with self.goal_lock:
            for goal in self._pending_goals.values():
                if goal['status']:
                    status_array.status_list.append(goal['status']);

        status_array.header.stamp = rospy.Time.now()
        self.status_topic.publish(status_array)

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

        rostype = None
        try:
            rostype = method._ros_action_type # Is it a ROS action?
            logger.info(component_name + "." + service_name + " is a ROS action of type " + str(rostype))
        except AttributeError:
            logger.info(component_name + "." + service_name + " has no ROS-specific action type. Skipping it.")
            return False

        self.actions.append(RosAction(self, component_name, service_name, rostype))

        logger.info("Created new ROS action server for {}.{}".format(
                                                    component_name,
                                                    service_name))

        return True


    def register_ros_service(self, method, component_name, service_name):
        
        # Default service type
        rostype = MorseAnyService
        
        try:
            rostype = method._ros_service_type # Is it a ROS service?
            logger.debug(component_name + "." + service_name + " is a ROS service of type " + str(rostype))
        except AttributeError:
            logger.debug(component_name + "." + service_name + " has no ROS-specific service type. Using default one.")
        
        cb = self.add_ros_handler(component_name, service_name)

        s = rospy.Service(component_name + "/" + service_name, rostype, cb)
        logger.debug("Created new ROS service for {}.{}".format(
                                                    component_name,
                                                    service_name))

        return True
    
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
            return self.register_ros_action(method, component_name, service_name)
        else:
            return self.register_ros_service(method, component_name, service_name)

    def add_ros_handler(self, component_name, service_name):
        """ Dynamically creates custom ROS->MORSE dispatchers
        for ROS *services* only.

        ROS actions are dealt with in the :py:class:`RosAction` class.
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
            if not is_sync:
                logger.error("Internal error: This ROS "
                             "service is bound to an asynchronous MORSE"
                             " service! ({})".format(component_name + '.' + service_name))
                raise rospy.service.ServiceException("MORSE Internal error! Check MORSE logs for details.")


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
        # First, figure out which 'ROSAction' manages this request id:
        manager = None

        for action in self.actions:
            if action.manage_internal_id(request_id):
                manager = action
                break

        if manager == None:
            logger.error("A ROS action call has been lost! Nobody manage request " + request_id)

        # Then, dispatch the 'on completion' event.
        status, value = result
        manager.on_result(request_id, status, value)

    def main(self):
        pass

class MorseAnyService(object):
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
      Your ROS action will appear as /component_instance/<name>/...
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
