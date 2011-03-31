Sockets
=======


Data-stream interface
---------------------

A simple method to export data through the network. The MORSE sockets
middleware will create a single UDP port for each robot, and the communication
for all components of the robot using sockets will be made through the same
port.

Data is shared as simple text strings.

The socket data-stream interface is implemented in :py:mod:`morse.middleware.socket_mw`.

.. note:: The port numbers used for the socket datastream interface start at 60000.

.. warning::
    Because of `bug 162 <https://softs.laas.fr/bugzilla/show_bug.cgi?id=162>`_,
    the socket interface does not currently allow for *outbound* connection without a
    prior *inbound* connection from the client.
    

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
  arguments. Arguments must be enclosed in brackets.

MORSE answer follows this model::

  id status [result]

- ``id`` the same id the client used to send the request,
- ``status``: one of the :py:mod:`morse.core.status` constants
- ``result``: a JSON-like result (actually, the representation of the Python
  result object), if any.

Example::

  $ telnet localhost 4000
  Connected to localhost.
  > req1 Human move (1.0, 2.0)
  req1 OK

.. note:: The socket service interface listen by default on port 4000.

.. note:: Why 4000?? That's all the question! A free beer for the first who finds.

The socket service interface is implemented in :py:mod:`morse.middleware.socket_request_manager`.

Files
-----

- Blender: ``$MORSE_ROOT/data/morse/components/middleware/socket_empty.blend``
- Python: ``$MORSE_ROOT/src/morse/modifiers/socket_mw.py``

Available methods
-----------------

- ``read_message``: Reads data as a pickled_ Python dictionary into the
  ``local_data`` associated to the component. The dictionary keys must
  be identical to the component ``local_data`` keys.
- ``post_message``: Dumps a pickled_ version of the tuple ``(component_name, local_data)`` on the socket.
  It can be read on the other end with ``pickle.loads``.

.. _pickled: http://docs.python.org/library/pickle.html
