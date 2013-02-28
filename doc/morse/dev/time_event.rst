Time and event in MORSE
=======================

Events during a simulation
--------------------------

Blender's Game Engine is set to work at a predefined ``Tickrate``, which is the
number of times the Logic Brick events are executed per second. In the default
settings, it is equal to 60, so that during one real second there will be 60
"ticks".

While the simulation is running, the Logic Bricks of each component will make
regular calls to their ``default_action`` method. At this point the component
will perform its task and update its internal data.

To run a component at a lower frequency some calls to ``default_action`` can
be skipped by setting the frequency in the Game Logic Sensor or more
conveniently specifying the desired frequency in the builder script.
The real frequency the ``default_action`` is called at can be read from the
``frequency`` property of the component.

Time management
---------------

The simulator will measure the amount of real time that has passed since the
Game Engine was started. Time is measured in seconds, and stored as a floating
point value with two decimals. It is obtained using the Python method:
``time.clock()``, which relies on the C function of the same name.
Simulation time is accessible from any Python script that imports the
``bge.logic`` module, by reading the variable: ``bge.logic.current_time``.
