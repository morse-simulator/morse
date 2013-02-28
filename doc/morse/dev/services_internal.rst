Understanding service internals
===============================

Registering synchronous services
--------------------------------

What exactly happens when a method is decorated with ``@service``?

The ``@service`` decorator simply marks the method as a service by
setting the attribute ``_morse_service`` to ``True`` on the function.

Before actually registering the service, a mapping between the component
and one or several middlewares that will manage incoming requests must
be defined (for instance, we may want the ``goto`` service of the
:py:class:`morse.actuators.waypoint.Waypoint` to be managed
by the YARP middleware for the component ``MainRobot``).

These mapping are defined by your builder script (it is so
simulation-dependant).

At start up, :py:func:`morse.blender.main.init`...

1. Reads the configuration, 
2. Instantiates classes associated to each component, 
3. Registers the mappings (with 
   :py:meth:`morse.core.services.MorseServices.register_request_manager_mapping`),
4. Calls :py:meth:`morse.core.abstractobject.AbstractObject.register_services`
   on each component instance.

:py:meth:`morse.core.abstractobject.AbstractObject.register_services` iterates over
every methods marked as MORSE service within the class, and call
:py:func:`morse.core.services.do_service_registration`.

This method finds the middleware(s) in charge of managing services for
this component, and calls
:py:meth:`morse.core.request_manager.RequestManager.register_service`.

It is up to each middleware to manage registration of new services. They
may have to, for instance, expose the new service into a shared directory
(*yellow pages*), etc.

Upon incoming request...
------------------------

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
---------------------

Registration of asynchronous services is mostly identical to synchronous
services. The ``@async_service`` decorator simply calls the ``@service``
decorator with the ``async`` parameter set to ``True``, which leads to
wrap the original method in a new method that takes one more parameter
(a callback) and calls
:py:meth:`morse.core.abstractobject.AbstractObject.set_service_callback`.

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
* When :py:meth:`morse.core.abstractobject.AbstractObject.completed`
  is invoked (i.e., when the service is completed), the callback 
  is executed.
* This causes in turn the 
  :py:meth:`morse.core.request_manager.RequestManager.on_service_completion`
  method to be invoked, to notify middleware-specific request 
  managers that the task is complete.

.. _manually-registring-services:

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
 
