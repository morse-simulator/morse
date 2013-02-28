Time and event in MORSE
=======================

Events during a simulation
--------------------------

Blender's Game Engine is set to work at a predefined ``Tickrate``, which is the
number of times the Logic Brick events are executed per second. In the default
settings, it is equal to 60, so that during one real second there will be 60
"ticks". It can be modified for a scene with the builder API using the method
:py:meth:`morse.builder.environment.Environment.set_speed`.

While the simulation is running, the Logic Bricks of each component will make
regular calls to their ``default_action`` method. At this point the component
will perform its task and update its internal data.

To run a component at a lower frequency some calls to ``default_action`` can
be skipped by setting the frequency in the Game Logic Sensor or more
conveniently specifying the desired frequency in the builder script (using
:py:meth:`morse.builder.abstractcomponent.AbstractComponent.frequency`). The
real frequency the ``default_action`` is called can be computed using the
properpy :py:meth:`morse.core.object.Object.frequency`.

Time management
---------------

The simulator will measure the amount of real time that has passed since the
Game Engine was started. Time is measured in seconds, and stored as a floating
point value with two decimals. It is obtained using the Python method:
:py:meth:`time.clock`, which relies on the C function of the same name.
Simulation time is accessible from any Python script in Morse that imports the
by reading the variable
:py:data:`morse.core.blenderapi.persistantstorage.current_time`.
