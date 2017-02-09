MOOS and MORSE tutorial :tag:`moos`
===================================

This is an extension of the current :doc:`tutorial <../beginner_tutorials/tutorial>`
using MOOS as our middleware (instead of raw sockets).

The builder script is the same as the one used in the sockets tutorial.
Except the part on the interface, which in this case is MOOS.  You can find it
in ``examples/tutorials/tutorial-1-moos.py``.


Configuring MOOS
----------------

.. code-block:: python

   gyroscope.add_stream('moos')
   motion.add_stream('moos')


Running the simulation
----------------------

Start a MOOS DB instance with::

    MOOSDB

In another termianl, run morse with::

    morse run examples/tutorials/tutorial-1-moos.py

``gyroscope`` will then publishes its data ``MORSE_GYRO_YAW``,
``MORSE_GYRO_YAW``, and``MORSE_GYRO_YAW`` to the `MOOSDB`.
``motion`` will be waiting for: ``MORSE_MOTION_VELOCITY`` or
``MORSE_MOTION_YAWRATE``to drive the vehicle.
One cans verify that all is working by scoping and poking the `MOOSDB`
