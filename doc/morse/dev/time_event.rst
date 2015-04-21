Time and event in MORSE
=======================

Events during a simulation
--------------------------

Blender's Game Engine is set to work at a predefined ``Tickrate``, which is the
number of times the Logic Brick events are executed per second. In the default
settings, it is equal to 60, so that during one real second there will be 60
"ticks". It can be modified for a scene with the builder API using the method
:py:meth:`morse.builder.bpymorse.set_speed`.

.. warning:: This method must be called at the top of your Builder script,
  before creating any component.

While the simulation is running, the Logic Bricks of each component will make
regular calls to their ``default_action`` method. At this point the component
will perform its task and update its internal data.

To run a component at a lower frequency some calls to ``default_action`` can
be skipped by setting the frequency in the Game Logic Sensor or more
conveniently specifying the desired frequency in the builder script (using
:py:meth:`morse.builder.abstractcomponent.AbstractComponent.frequency`). The
real frequency the ``default_action`` is called can be computed using the
property :py:meth:`morse.core.object.Object.frequency`.

Time management
---------------

At the moment, there are two strategies for handling time at the Morse level:

- the **Best effort** strategy try to handle your requirement in "real-time",
  i.e. one second of time in simulation is equal to one second of time in the
  real world. To do that, Morse may need to drop some frames. Simulator may be
  less accurate too. The "good" point is that you do not need to care too much
  about time. It is the default mode.

- the **Fixed Simulation Step** strategy handles all physical / logical steps,
  at fixed simulation step. It means that between each step, the simulation
  acts as if '1 / frequency' sec has elapsed. The simulation is so more
  accurate, but simulated time may diverge from real time.

These different strategies are implemented in :py:mod:`morse.core.morse_time`.

Since 1.3, it is also possible to synchronise the simulation step, and so the
time with an external clock for a 'slower than real-time' simulation. This
synchronisation mechanism makes sense only in **Fixed Simulation Step** or in
the case where you don't really use the physical engine. For the moment, this
mechanism is implemented for two middlewares (search for the **time_sync**
parameter):

- :doc:`../user/middlewares/socket`
- :doc:`../user/middlewares/hla`

The used strategy is selected at the builder level, through the method
:py:meth:`morse.builder.environment.Environment.set_time_strategy`.

In the simulator itself, you can access to the simulated time via
:py:data:`morse.core.blenderapi.persistantstorage().time.time`. It returns the
simulated time as the number of seconds (in float) since Epoch, as done by
:py:meth:`time.time`. More precisely, at startup, the simulated is initialized
with :py:meth:`time.time` and then progress depending of the selected
strategy. The precision depends of the underlaying implementation of
:py:meth:`time.time` and the speed of simulation. If you run a simulation at
60 Hz, the simulator clock will be updated about every 15 ms.

.. note::

    The variable
    py:data:`morse.core.blenderapi.persistantstorage().current_time` still
    exists, for compatibility purpose, but will be removed in the future.


Moreover, in a lot of situations, you do not want to access the
simulated time directly, but at the time as seen by the current robot. To do that, you
must call the method :py:meth:`morse.core.robot.Robot.gettime`. It allows
different modifiers to be added for different robots, triggering all the nice temporal
issues you must address in multi-robot situations. The
:doc:`../user/sensors/clock` exposes the time, as seen by a specific
robot.

Last, a set of services in :py:mod:`morse.services.time_services` allows to
retrieve the simulated time and various statistics about it.

