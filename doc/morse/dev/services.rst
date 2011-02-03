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
       # This is normaly called by MORSE in its main loop.
       req_manager.process()

If you run this sample code, you can test it with a simple Telnet session::

  > telnet localhost 60001
  Connected to localhost.
  > req1 test_component add (1,2)
  req1 OK 3

.. note::
   The socket-based protocol is fairly simple: you provide a request id, the
   name of the component that offer the service, the name of the service and
   (if necessary) parameters. Parameters must be contained in a valid Python
   iterable (a tuple, like in the example, or an array).

   Here, ``req1`` is the custom request id, chosen by the client.

For asynchronous services, a callback function is passed to the service. It
allows the service to notify when it is complete.

This second code snippet shows an example of asynchronous service:

.. code-block:: python

   import time
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
           result_cb("done!")
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

  > telnet localhost 60001
  Connected to localhost.
  > req2 test_component compute (5,)
  [after 5 seconds]
  req2 OK done!

  .. note::
    When passing a single parameter, you still need to pass a valid Python iterable,
    with only one element.  Hence the ``(5,)``.
 
Registration of services in a component
---------------------------------------

TODO
