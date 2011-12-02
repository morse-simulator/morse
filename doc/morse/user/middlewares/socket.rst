Sockets
=======


Data-stream interface
---------------------

A simple method to export data through the network. The MORSE sockets
middleware will create one socket for each component. The service
``simulation.list_streams'' returns the list of available data streams, and the
service ``simulation.get_stream_port(<stream name>)`` returns the port number
associated to this stream.

Data is shared as JSON object, encoded in utf-8 strings.

The socket data-stream interface is implemented in :py:mod:`morse.middleware.socket_mw`.

.. note:: The port numbers used for the socket datastream interface start at 60000.


Service interface
-----------------

Requests to components or MORSE itself can be sent through the socket interface.

The (ASCII) protocol is simple::

  id component service [parameters]

- ``id`` is a freely chosen request id. It is mainly useful to identify answers
  from asynchronous services.  
- ``component`` is the name of the component you want to invoke the service on.

.. note::
  Services that control the whole simulator belongs to the special component ``simulation``.

- ``service``: the name of the request to invoke.
- ``parameters`` (can be omitted if the request takes no argument): request
  arguments in JSON format. Arguments must be enclosed in a list (ie, inside
  brackets).

MORSE answer follows this model::

  id status [result]

- ``id`` the same id the client used to send the request,
- ``status``: one of the :py:mod:`morse.core.status` constants
- ``result``: a JSON-serialized result, if any.

Example::

  $ telnet localhost 4000
  Connected to localhost.
  > req1 Human move [1.0, 2.0]
  req1 OK

.. note:: The socket service interface listen by default on port 4000. If this
	port is busy, MORSE will try to connect to the next 10 ports {4001-4010}
	before giving up.

.. note:: Why 4000?? That's all the question! A free beer for the first who finds.

The socket service interface is implemented in :py:mod:`morse.middleware.socket_request_manager`.

Files
-----

- Blender: ``$MORSE_ROOT/data/morse/middleware/socket_mw.blend``
- Python (data-stream): ``$MORSE_ROOT/src/morse/middleware/socket_mw.py``
- Python (services): ``$MORSE_ROOT/src/morse/middleware/socket_request_manager.py``

Available methods
-----------------

- ``read_message``: Reads data as a json dictionary into the
  ``local_data`` associated to the component. The dictionary keys must
  be identical to the component ``local_data`` keys.
- ``post_message``: Dumps a json version of the component ``local_data`` on the socket.
  It can be read on the other end with ``json.loads``.

.. _json: http://docs.python.org/library/json.html
