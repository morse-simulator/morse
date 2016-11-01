Sockets
=======


Data-stream interface
---------------------

This provides a simple way to export data through the network. The MORSE sockets
middleware will create one socket for each component. The service
``simulation.list_streams`` returns the list of available data streams, and the
service ``simulation.get_stream_port(<stream name>)`` returns the port number
associated with this stream.

Data is shared as JSON objects, encoded in UTF-8 strings. As such, when
exchanging data with a particular component using this middleware, it is
necessary to create a similar data structure to the component's
``local_data`` dictionary.
For example, when using telnet to connect to a
:doc:`waypoint actuator <../actuators/waypoint>`, you need to send a message
like the following::

  $ telnet localhost 60000
  {"x":3.0, "y":5.0, "z":0.0, "tolerance":0.5, "speed":2.0}

The socket data-stream interface is implemented in :py:mod:`morse.middleware.socket_datastream`.

.. note:: The port numbers used for the socket datastream interface start at 60000.

.. _socket_ds_configuration:

Configuration specifics
-----------------------

When configuring a component to export its data through a socket, you can
specify the ``port`` that you want the socket to listen to. Note that this
is only a "hint", in some cases, it is not possible to satisfy all the
constraints.

.. code-block :: python

    foo.add_stream('socket', port = 60005)

It is also possible to enable a time synchronisation mechanism using
the following parameters:

- **time_sync**: Optional: Enable or disable the time synchronisation
  mechanism. The default value is False
- **sync_port**: Optional: It is the port where the simulator waits for the
  signal for synchronisation. The default value is 6000.

To enable time synchronisation using a custom port, say 12000, you can do:

.. code-block :: python

    env.configure_stream_manager('socket', time_sync = True, sync_port = 12000)

If the time synchronisation mechanism is enabled and once someone is connected
to the synchronisation port, the simulator will wait for a message (any
string of length < 2048) at each turn of the simulation. Once the client
disconnects, the simulator is then freed to run at "normal" speed.


Service interface
-----------------

Requests to components or MORSE itself can be sent through the socket interface.

The (ASCII) protocol is simple. Either::

  id component service [parameters]

or::

  id special

- ``id`` is a freely chosen request id. It is mainly useful to identify answers
  from asynchronous services.  
- ``component`` is the name of the component you want to invoke the service on.

.. note::
  Services that control the whole simulator belong to the special component ``simulation``.

- ``service``: the name of the request (method) to invoke.
- ``parameters``: only required if the request requires one or more
  arguments, in which case it must be a JSON-format list.
- ``special``: a special command, used to manipulate already existing requests.
  Currently, the only special command is ``cancel`` (to abort a running
  service).

MORSE responses are of this form::

  id status [result]

- ``id`` the same id the client used to send the request,
- ``status``: one of the :py:mod:`morse.core.status` constants
- ``result``: a JSON-serialized result, if any.

Example::

  $ telnet localhost 4000
  Connected to localhost.
  > req1 Human move [1.0, 2.0]
  req1 OK

.. note:: By default the socket service interface listens on port 4000. If this
	port is busy, MORSE will try to connect to the next 10 ports {4001-4010}
	before giving up.

The socket service interface is implemented in :py:mod:`morse.middleware.socket_request_manager`.

Files
-----

- Python (data-stream): ``$MORSE_ROOT/src/morse/middleware/socket_datastream.py``
- Python (services): ``$MORSE_ROOT/src/morse/middleware/socket_request_manager.py``

.. _json: http://docs.python.org/library/json.html
