GPS noise
===========

This modifier adds a random gaussian noise to the variables in the ``local_data``
of the linked component.
It is designed to be used with the :doc:`GPS component <../sensors/gps>`, but could be applied to other
components that have only numerical data values in their ``local_data``.
If it finds non numerical values, it will generate an error.
It utilises a module programmed in C.

Files
-----

- Python: ``$MORSE_ROOT/src/morse/modifiers/gps_noise.py``

Modified data
-------------

Modifiers work over a dictionary called ``local_data`` of the components.
It will try to add noise to all variables in the dictionary, so it should
only be applied to components that use only numerical data.


Available methods
-----------------

- ``noisify``: Use to add gaussian noise to the data of a component
