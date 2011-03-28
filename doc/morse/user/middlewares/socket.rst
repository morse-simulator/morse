Sockets
=======

A simple method to export data through the network. The MORSE sockets
middleware will create a single UDP port for each robot, and the communication
for all components of the robot using sockets will be made through the same
port.

Data is shared as simple text strings.

.. note:: The port numbers used in MORSE start at 60000.

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
