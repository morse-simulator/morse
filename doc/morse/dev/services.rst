Services in MORSE
=================

MORSE has a concept of *services*: services are remote procedure call commonly
used to configure or control the behaviour of the simulation.

Each component can register services that are made publicly available to the
outside world through middleware-specific channels.

Services can be either *synchronous* or *asynchronous*. Synchronous services
block the simulation until they complete. They must remain fast to execute.
Asynchronous services may span computations on several simulation steps (but
each individual cycle must remain fast).

Adding new services
-------------------

Exposing methods as services
++++++++++++++++++++++++++++

Most of the time, adding a new service is as easy as adding ``@service``
in front of a function declared within a component.

Lets have a look at ``human.py``, the component that allows us to control
a human character in the simulation.

.. code-block:: python

    import morse.core.robot
    from morse.core.services import service

    class HumanClass(morse.core.robot.MorseRobotClass):

        def __init__(self, obj, parent=None):
            [...]
 
        @service
        def move(self, speed, rotation):
            
            human = self.blender_obj
            
            human.applyMovement( [speed,0,0], True )
            human.applyRotation( [0,0,rotation], True )

        [...]

By adding the ``@service`` decorator to the ``move`` method, we expose
``move`` as a MORSE service.

During the simulation initialization, MORSE registers these services for
each instances of the component, maps them to one (or several)
middlewares (as specified by the user in :doc:`component_config.py <../user/hooks>`), and
starts listening for incoming request.

Each middleware has its own naming scheme for services, but one can
expect the services to be available as ``component_name.service_name``.

The example below shows a simple Python client that would use the
``HumanClass.move`` service as declared above:

.. code-block:: python

  import socket
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.connect(("localhost", 4000))
  s.send("id1 myHuman move (1.0, 1.6)\n")

In this example, we assume that ``myHuman`` is the name of the Blender
object that instanciates a ``HumanClass``.

.. note::
  The value of the id (here ``id1``) has no importance at all: it is
  defined and used only by the client to track requests and responses.

Returning values
++++++++++++++++

A service can return any valid Python object (``None``, a string, a
dictionary, a complex object...). The serialization is left to the
middleware.

If the service call fails, you are expected to raise a
:class:`morse.core.exceptions.MorseRPCInvokationError` exception
(or any custom exception inheriting from it) with a useful error message:

.. code-block:: python

    import morse.core.robot
    from morse.core.exceptions import MorseRPCInvokationError
    from morse.core.services import service

    class HumanClass(morse.core.robot.MorseRobotClass):

        def __init__(self, obj, parent=None):
            [...]
 
        @service
        def move(self, speed, rotation):
            
            if speed < 0:
                raise MorseRPCInvokationError("Our human can not walk backward!")

            human = self.blender_obj
            
            human.applyMovement( [speed,0,0], True )
            human.applyRotation( [0,0,rotation], True )

        [...]

*MORSE* will answer the request with a
:data:`morse.core.status.FAILED` status.

Free functions
++++++++++++++

Synchronous services can also be declared outside classes (on
free-functions).

In this case, the decorator takes one parameter, the (pseudo) component.

For instance, ``morse.core.supervision_services.py`` declares such
services. The following example shows the ``list_robots`` service that
returns the list of robot declared in the simulation:

.. code-block:: python

    import GameLogic
    from morse.core.services import service

    @service(component = "simulation")
    def list_robots():
        return [obj.name for obj in GameLogic.robotDict.keys()]

The pseudo-component ``simulation`` is used as *namespace* for the
service: this one is accessible as ``simulation.list_robots``.

Asynchronous services
+++++++++++++++++++++

RPC calls may be used to start the execution of a task that may take a
long time to complete.

In such cases, **asynchronous services** can be used to initialize and start
the task. MORSE automatically notifies the client when the task is
completed.

Declaring new asynchronous services is slightly more complex: we need
first an *initialization method* and secondly, a way to tell when the
task is achieved.

Declaring an initialization method is very similar to synchronous
services. For instance, the *waypoint* actuator defines an asynchronous
``goto`` service:

.. code-block:: python

    import morse.core.actuator
    from morse.core.services import async_service

    class WaypointActuatorClass(morse.core.actuator.MorseActuatorClass):

        def __init__(self, obj, parent=None):
            [...]

        @async_service
        def goto(self, x, y):
            self.local_data['x'] = x
            self.local_data['y'] = y
            self.local_data['z'] = 0 

        [...]

The ``@service`` decorator is simply replaced by ``@async_service``. By
doing so, MORSE automatically registers a callback that is used to
monitor the status of the task and notify the client upon completion.

In this example we simply set a new target position in the actuator
``local_data`` dictionary, but any kind of initialization can be started
here. It must only remain short (since the simulator blocks until the
initialization method returns).

The execution of the task itself takes place at each simulation step in
the component
:py:meth:`morse.core.object.MorseObjectClass.default_action` method.
Each execution step should remain short since the simulator blocks on
the ``default_action`` as well.

When the task is achieved, the component must notify it by calling
``self.completed(status, result)``.

``status`` is one of the status defined in :py:mod:`morse.core.status`
(mainly ``SUCCESS`` and ``FAILED``), ``result`` is any valid Python
object.

.. note::
  As you may have noticed, at a given time, only one asynchronous
  request can be handled by a component.  If a second asynchronous
  request is received, the behaviour may vary, as explained below.

.. note::
  Asynchronous services can normally only exist inside components (i.e.,
  they must be declared within a class inheriting from
  :py:class:`morse.core.abstractobject.MorseAbstractObject`).
  The section `Manually registering services`_ explains how to overcome
  this constraint.

Interruption policy for asynchronous services
+++++++++++++++++++++++++++++++++++++++++++++

As of ``morse-0.4``, only one asynchronous service may run at a given time.

You can define the behaviour of the simulator when a second request is received
either at the middleware level (*global policy*) or at the individual service
level (*local policy*).

To set a local policy, simply decorate your services with the
``@interruptible`` and ``@noninterruptible`` decorators
(:meth:`morse.core.services.interruptible` and
:meth:`morse.core.services.noninterruptible`).

An *interruptible* service is preempted (ends with status
:data:`morse.core.status.PREEMPTED`) when a new asynchronous service is
started. A *non-interruptible* service triggers a failure when someone attempts
to start a new service.


To set a global policy, you need to catch a
:class:`morse.core.exceptions.MorseServiceAlreadyRunningException` exception
when invoking the :meth:`morse.core.request_manager.on_incoming_request`
method.

This exception has a special member ``service`` that points to the original
service function:

.. code-block:: python

    try:
        is_synchronous, value = self.on_incoming_request(component, service, params)
    except MorseServiceAlreadyRunningError as e:
        logger.warning(e.service.__name__ + " is already running!")

The internals
-------------

Registering synchronous services
++++++++++++++++++++++++++++++++

What exactly happens when a method is decorated with ``@service``?

The ``@service`` decorator simply marks the method as a service by
setting the attribute ``_morse_service`` to ``True`` on the function.

Before actually registering the service, a mapping between the component
and one or several middlewares that will manage incoming requests must
be defined (for instance, we may want the ``goto`` service of the
:py:class:`morse.actuators.waypoint.WaypointActuatorClass` to be managed
by the YARP middleware for the component ``MainRobot``).

These mapping are defined in the :doc:`component_config.py <../user/hooks>`
script (that is simulation-dependent).

At start up, :py:func:`morse.blender.main.init`...

1. Reads the configuration, 
2. Instantiates classes associated to each component, 
3. Registers the mappings (with 
   :py:meth:`morse.core.services.MorseServices.register_request_manager_mapping`),
4. Calls :py:meth:`morse.core.object.MorseObjectClass.register_services`
   on each component instance.

:py:meth:`morse.core.MorseObjectClass.register_services` iterates over
every methods marked as MORSE service within the class, and call
:py:func:`morse.core.services.do_service_registration`.

This method finds the middleware(s) in charge of managing services for
this component, and calls
:py:meth:`morse.core.request_manager.RequestManager.register_service`.

It is up to each middleware to manage registration of new services. They
may have to, for instance, expose the new service into a shared directory
(*yellow pages*), etc.

Upon incoming request...
++++++++++++++++++++++++

When a new request comes in, the middleware-specific part receives it,
deserializes it and hands it over to
:py:meth:`morse.core.request_manager.RequestManager.on_incoming_request`.
This method dispatches the request to the correct component, and executes
it (for synchronous services) or starts the execution and returns an
internal request ID (for asynchronous services).

This internal request ID can be used by the middleware to track the
status of a request.

On completion, the
:py:meth:`morse.core.request_manager.RequestManager.on_service_completion`
callback is invoked, and the final result can be sent back to the
client.

Asynchronous services
+++++++++++++++++++++

Registration of asynchronous services is mostly identical to synchronous
services. The ``@async_service`` decorator simply calls the ``@service``
decorator with the ``async`` parameter set to ``True``, which leads to
wrap the original method in a new method that takes one more parameter
(a callback) and calls
:py:meth:`morse.core.object.MorseObjectClass.set_service_callback`.

Simplified version of the ``@service`` decorator:

.. code-block:: python

    def service(fn, async=False):
      dfn = fn
      if async:
         def decorated_fn(self, callback, *param):
            self._set_service_callback(callback)
            fn(self, *param)
         dfn = decorated_fn
         dfn.__name__ = fn.__name__

      dfn._morse_service = True
      dfn._morse_service_is_async = async

      return dfn

However, asynchronous services behaviour differs when a request comes
in:

* :py:meth:`morse.core.request_manager.RequestManager.on_incoming_request`
  creates a new callback function for this service,
* It invokes the original method with this callback,
* When :py:meth:`morse.core.object.MorseObjectClass.completed`
  is invoked (i.e., when the service is completed), the callback 
  is executed.
* This causes in turn the 
  :py:meth:`morse.core.request_manager.RequestManager.on_service_completion`
  method to be invoked, to notify middleware-specific request 
  managers that the task is complete.

Manually registering services
-----------------------------

While usually not necessary, you may have sometimes to manually register
services (i.e. without using decorators).

This first code snippet shows how to register a synchronous service that uses
sockets as communication interface:

.. code-block:: python

   from morse.middleware.socket_request_manager import SocketRequestManager
  
   def add(a, b):
       return a + b

   req_manager = SocketRequestManager()
   req_manager.register_service("test_component", add)

   while True:
       # This calls the middleware specific part, in charge of reading
       # incoming request and writing back pending results.
       req_manager.process()
       # In a real case, you don't want to block on such a loop, and MORSE
       # takes care itself to call req_manager.process()

If you run this sample code, you can test it with a simple Telnet session::

  > telnet localhost 4000
  Connected to localhost.
  > req1 test_component add (1,2)
  req1 OK 3

.. note::
   The socket-based protocol is fairly simple: you provide a request id, the
   name of the component that offers the service, the name of the service and
   (if necessary) parameters. Parameters must be contained in a valid Python
   iterable (a tuple, like in the example, or an array).

   Here, ``req1`` is the custom request id, chosen by the client.

For asynchronous services, a callback function is passed to the service. It
allows the service to notify when it is complete.

This second code snippet shows an example of asynchronous service:

.. code-block:: python

   import time
   from morse.core import status
   from morse.middleware.socket_request_manager import SocketRequestManager
   
   result_cb = None
   run_computation = False
   value = None

   # We start here our asynchronous service.
   # an arbitrary amount of parameters can be passed, but the first one
   # must be the callback to set the service result upon completion.
   def start_computation(result_setter, start_value):
        global result_cb, value, run_computation

        result_cb = result_setter
        value = start_value
        run_computation = True

        # the service must return true if the task was successfully started
        return True

   # This is the actual code called at each simulation step in our component
   def complex_computation(a):
       global run_computation

       if a == 0:
           # At the end of the computation, we set the result.
           # the result can be any valid Python object
           result_cb((status.SUCCESS, "done!"))
           run_computation = False

       time.sleep(1)
       return a - 1

   req_manager = SocketRequestManager()

   # here we explicitely register an asynchronous service.
   # the optional 'service_name' argument allows to define a custom service
   # name.
   req_manager.register_async_service("test_component", start_computation, service_name = "compute")

   while True:
       req_manager.process()

       if run_computation:
          value = complex_computation(value)
          print("Value is now %i" % value)


If you test the code with Telnet::

  > telnet localhost 4000
  Connected to localhost.
  > req2 test_component compute (5,)
  [after 5 seconds]
  req2 OK done!

.. note::
    When passing a single parameter, you still need to pass a valid Python iterable,
    with only one element.  Hence the ``(5,)``.
 
