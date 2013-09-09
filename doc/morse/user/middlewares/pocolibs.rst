Pocolibs
========

Pocolibs middleware is mainly used with the robot architecture established at
LAAS. It relies heavily on the ctypes library, and for some more complex
poster, on 'hand-tuned C'.

For the moment, the Pocolibs middleware relies only on the concept of *posters*
to export and import data. It requires the use of the ``h2`` program to
manage the posters running in the computer.

Each of the posters has a very specific data structure that is used to store
and transfer the information. For this reason, there is no generic methods for
this middleware, and all of the methods are defined as extensions to the basic
functionality of the middleware.

Files
-----

- Python: ``$MORSE_ROOT/src/morse/modifiers/pocolibs_datastream.py``

.. _pocolibs_ds_configuration:

Configuration specificities
---------------------------

When configuring a component to export its data through pocolibs, you can pass
the option ``poster`` to define the name of the poster exported by the
component.


.. code-block :: python

    foo.add_stream('pocolibs', poster = 'myFooPoster')
