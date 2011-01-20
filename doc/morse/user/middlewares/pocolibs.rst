Pocolibs
========

This middleware is more complex than the others, since it requires the use of
C programming and SWIG to interact with Python. It is used mainly with the
robot architecture established at LAAS.

For the moment, the Pocolibs middleware relies only on the concept of *posters*
to export and import data. It requires the use of the ``h2`` program to
manage the posters running in the computer.

Each of the posters has a very specific data structure that is used to store
and transfer the information. For this reason, there is no generic methods for
this middleware, and all of the methods are defined as extensions to the basic
functionality of the middleware.


.. note:: The data structures used to transfer data with Pocolibs are those
  defined in the include files of the Genom modules. These include files
  must be listed in the header files of the C modules and then
  compiled using SWIG. 

Files
-----

- Blender: ``$MORSE_ROOT/data/morse/components/middleware/pocolibs_empty.blend``
- Python: ``$MORSE_ROOT/src/morse/modifiers/pocolibs_mw.py``

Available methods
-----------------

None

Available extensions
--------------------

These files contain additional methods that can be used with the Pocolibs middleware.
To use them, it is necessary to list the file as the third element of the middleware
list, in the ``component_config.py`` script, as described in the :doc:`hooks <../hooks>`
documentation.

- POM poster: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/pocolibs/sensors/pom.py``.
  It has one available output method:

    - ``write_pom``: Sends data of type **POM_ME_POS**, defined in ``pom/pomStruct.h``

- VIAM poster: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/pocolibs/sensors/viam.py``.
  It has one available output method:

    - ``write_viam``: Sends data of type **simu_image**, defined in ???. VIAM data stored in ``viam/viamStruct.h``

- VIMAN poster: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/pocolibs/sensors/viman.py``.
  It has one available output method:

    - ``write_viman``: Sends data of type **VimanObjectPublicArray**, defined in ``viman/vimanStruct.h``

- GENPOS poster: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/pocolibs/controllers/genpos.py``.
  It has one available input method:

    - ``read_genpos``: Reads data of type **GENPOS_CART_SPEED**, defined in ``genPos/genPosStruct.h``

- LWR poster: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/pocolibs/controllers/lwr.py``.
  It has one available input method:

    - ``read_lwr_config``: Reads data of type **LWR_ARM_INST**, defined in ``lwr/lwrStruct.h``

- PLATINE poster: Stored in the file: ``$MORSE_ROOT/src/morse/middleware/pocolibs/controllers/platine.py``.
  It has one available input method:

    - ``read_platine``: Reads data of type **POM_SE_POSTER**, defined in ``pom/pomStruct.h``

Known problems
--------------

Input posters (those that will be read by MORSE) must be created before launching the simulation.
Otherwise, they will not be found and the component that is bound to them will not function.
