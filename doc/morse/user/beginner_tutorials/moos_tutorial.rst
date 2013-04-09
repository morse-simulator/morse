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

Start uMS (requires ui-moos package) as a simple way to control the robot and
view its sensor output.  The ATRV can be controlled by publishing a
longitudinal speed in m/s to ``cVelocity`` and publishing a yaw rate in rad/sec
to ``cYawRate``.  The output of the Pose sensor should also be visible in uMS
under the ``zPitch``, ``zRoll``, and ``zYaw`` variables.
