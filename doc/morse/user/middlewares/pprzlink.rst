PPRZLINK
========

`PPRZLink <https://github.com/paparazzi/pprzlink>`_ is the communication system
used by the `Paparazzi UAV system <http://paparazziuav.org>`_.

Several interfaces are available with PPRZLink (OCaml, Python and C) as well
as several communication supports (RF modems, UDP, IVY). This MORSE
implementation is based on the Python API and the `IVY software bus
<http://www.eei.cena.fr/products/ivy/>`_ which is used in Paparazzi for the
communication between the ground agents. It is based on a publisher/subscriber
mechanism. The bindings are done with regular expressions and only text (ASCII)
data can be exchanged with this middleware.

Required packages (from system or from source):
- lxml
- ivy
- pprzlink

Files
-----

- Python: ``$MORSE_ROOT/src/morse/middleware/pprzlink_datastream.py``

.. _pprzlink_ds_configuration:

Configuration specificities
---------------------------

When configuring a component, you can pass several options to adjust
the messages binding.

The option ``ac_id`` allows to select a specific aircraft. Otherwise, the
binding is done on every aircraft, which might lead to inconsistent 
behavior when multiple aircraft are used but is fine when only one is used
at a time.
The option ``msg_name`` can overwrite the message name for the binding. This
can only be done if the same fields are available in the original message
and the custom one.

.. code-block :: python

    foo.add_stream('pprzlink', ac_id = '101')


