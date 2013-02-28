Adding new service
==================

MORSE has a concept of *services*: services are remote procedure call commonly
used to configure or control the behaviour of the simulation.

Each component can register services that are made publicly available to the
outside world through middleware-specific channels.

Services can be either *synchronous* or *asynchronous*. Synchronous services
block the simulation until they complete. They must remain fast to execute.
Asynchronous services may span computations on several simulation steps (but
each individual cycle must remain fast).

Exposing methods as services
----------------------------

Most of the time, adding a new service is as easy as adding ``@service``
in front of a function declared within a component.

.. note::
    Arguments coming from remote caller are passed to service using string, so
    be sure to convert your data in the expected type.

.. warning::
    Do not use ``eval()`` to convert your data from string to expected type, as
    this would cause a serious security threat. Use type specific functions
    (like ``int()``, ``float()``,...) instead.

Lets have a look at ``human.py``, the component that allows us to control
a human character in the simulation.

.. code-block:: python

    import morse.core.robot
    from morse.core.services import service

    class HumanClass(morse.core.robot.Robot):

        def __init__(self, obj, parent=None):
            [...]
 
        @service
        def move(self, speed, rotation):
            
            human = self.bge_object
            
            human.applyMovement( [float(speed), 0, 0], True )
            human.applyRotation( [0, 0 , float(rotation)], True )

        [...]

By adding the ``@service`` decorator to the ``move`` method, we expose
``move`` as a MORSE service.

During the simulation initialization, MORSE registers these services for
each instances of the component, maps them to one (or several)
middlewares (as specified by the scene description), and starts listening for
incoming request.

Each middleware has its own naming scheme for services, but one can
expect the services to be available as ``component_name.service_name``.

The example below shows a simple Python client that would use the
``HumanClass.move`` service as declared above:

.. code-block:: python

  import socket
  s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
  s.connect(("localhost", 4000))
  s.send("id1 human move (1.0, 1.6)\n")

In this example, we assume that ``human`` is the name of the Blender
object that instantiates a ``HumanClass``.

.. note::
  The value of the id (here ``id1``) has no importance at all: it is specific
  to the implementation of the services with sockets and used only by the
  client to track requests and responses.

Using pymorse, the previous code can be rewrite as:

.. code-block:: python
    
    from pymorse import Morse
    with Morse() as morse:
        morse.rpc('human', 'move', 1.0, 1.6)

Returning values
----------------

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

    class HumanClass(morse.core.robot.Robot):

        def __init__(self, obj, parent=None):
            [...]
 
        @service
        def move(self, speed, rotation):
            
            if float(speed) < 0:
                raise MorseRPCInvokationError("Our human can not walk backward!")

            human = self.bge_object
            
            human.applyMovement( [float(speed), 0, 0], True )
            human.applyRotation( [0, 0, float(rotation)], True )

        [...]

*MORSE* will answer the request with a
:data:`morse.core.status.FAILED` status.

Free functions
--------------

Synchronous services can also be declared outside classes (on
free-functions).

In this case, the decorator takes one parameter, the (pseudo) component.

For instance, :py:mod:`morse.services.supervision_services` declares such
services. The following example shows the ``list_robots`` service that
returns the list of robot declared in the simulation:

.. code-block:: python

    from morse.core import blenderapi
    from morse.core.services import service

    @service(component = "simulation")
    def list_robots():
        return [obj.name for obj in blenderapi.persistantstorage().robotDict.keys()]

The pseudo-component ``simulation`` is used as *namespace* for the
service: this one is accessible as ``simulation.list_robots``.

Asynchronous services
---------------------

RPC calls may be used to start the execution of a task that may take a
long time to complete.

In such cases, **asynchronous services** can be used to initialize and start
the task. MORSE automatically notifies the client when the task is
completed.

Declaring new asynchronous services is slightly more complex: we need
first an *initialization method* and secondly, a way to tell when the
task is achieved.

Declaring an initialization method is very similar to synchronous
services. For instance, the :doc:`waypoint <../user/actuators/waypoint>`
actuator defines an asynchronous ``goto`` service:

.. code-block:: python

    import morse.core.actuator
    from morse.core.services import async_service

    class Waypoint(morse.core.actuator.Actuator):

        def __init__(self, obj, parent=None):
            [...]

        @async_service
        def goto(self, x, y):
            self.local_data['x'] = float(x)
            self.local_data['y'] = float(y)
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
:py:meth:`morse.core.object.Object.default_action` method.
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

  Asynchronous services can normally only exist inside components (*i.e.*,
  they must be declared within a class inheriting from
  :py:class:`morse.core.abstractobject.AbstractObject`).
  The section :ref:`manually-registring-services` explains how to overcome
  this constraint.

Interruption policy for asynchronous services
---------------------------------------------

As of ``morse-1.0``, only one asynchronous service may run at a given time.

You can define the behaviour of the simulator when a second request is received
either at the middleware level (*global policy*) or at the individual service
level (*local policy*).

To set a local policy, simply decorate your services with the
``@interruptible`` and ``@noninterruptible`` decorators
(:meth:`morse.core.services.interruptible` and
:meth:`morse.core.services.noninterruptible`). These decorators must appear
*before* the ``@async_service`` decorator.

An **interruptible** service is preempted when a new asynchronous service is
started by calling the ``interrupt`` method. The ``interrupt`` method is
defined in :py:class:`morse.core.abstractobject.AbstractObject` to send back
to the caller the status :data:`morse.core.status.PREEMPTED`. It is advised to
overload this behaviour in the component class to ensure the service is
actually interrupted (do not forget however to call overloaded ``interrupt``
method, as shown in the example below). 

.. code-block:: python

    import morse.core.actuator

    class Waypoint(morse.core.actuator.Actuator):

         def interrupt(self):
             self.local_data['x'] = self.bge_object.worldPosition[0]
             self.local_data['y'] = self.bge_object.worldPosition[1]
             self.local_data['z'] = self.bge_object.worldPosition[2]
             super(Waypoint, self).interrupt()

.. note::
    It is recommended to always implement the ``interrupt`` method even if the
    default policy is *non-interruptible*, as a caller can decide to manually
    interrupt the service.

A **non-interruptible** service triggers a failure (status
:data:`morse.core.status.FAILED`) when someone attempts to start a new
asynchronous service. 

To set a global policy, you need to catch a
:py:class:`morse.core.exceptions.MorseServiceAlreadyRunningError` exception
when invoking the :meth:`morse.core.request_manager.RequestManager.on_incoming_request`
method.

This exception has a special member ``service`` that points to the asynchronous 
service currently running:

.. code-block:: python

    try:
        is_synchronous, value = self.on_incoming_request(component, service, params)
    except MorseServiceAlreadyRunningError as e:
        logger.warning(e.service.__name__ + " is already running!")

.. note::
  A service with a local policy defined (*i.e.* decorated with either
  ``@interruptible`` or ``@noninterruptible``) will never trigger a
  ``MorseServiceAlreadyRunningError`` exception, and thus, **the local
  policy always overrides the global policy**.
