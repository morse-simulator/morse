Time and event in MORSE
=======================

This page presents the inner working of time in MORSE. If you are not yet
familiar with time issues and you simply want to configure the time-related
settings of your simulation, you should start :ref:`here <configure_time>`.

Understand time handling in the Blender's Game Engine
-----------------------------------------------------

A pseudo-code describing the behaviour of the Blender's Game Engine (for
Blender < 2.78 at least) is the following:

.. code-block:: pascal

    # main loop
    while not_quit:
        # logic / physics loop
        for i in 0..n:
            execute_logic
                ...
                synchronise_with_external_clock # optionally
                ...
                ...
            execute_physics
        end

        execute_graphics # v-sync occurs here if enabled

The loop is centered around the graphic update. By default in Blender, `v-sync
<https://en.wikipedia.org/wiki/Screen_tearing>`_ is enabled, so the main loop
is caped by the frequency of your screen (often 60 Hz). It is possible to
disable v-sync (this lets MORSE reach higher frequencies, assuming your
hardware permits) using
:py:meth:`morse.builder.environment.Environment.use_vsync`.  Though, as shown
in the pseudo-code, it is possible to run the logic / physics at higher
frequency. This behaviour can be configured using the builder API using the
method
:py:meth:`morse.builder.environment.Environment.simulator_frequency`. For
instance, ``env.simulator_frequency(20)`` means that the main loop will run at
20Hz (if your hardware is powerful enough). The ``base_frequency`` is the
frequency of the main loop by second. It will be adjusted automatically if you
try to accelerate or slow-down the time (see below). The ``logic_step_max``
and ``physics_step_max`` are used to compute the maximum possible ``n`` in the
previous loop.

While the simulation is running, during the ``execute_logic`` step, components
actually invoke their `default_action` method via the Blender *logic bricks*.
At this point the component will perform its task and update its internal
data.

To run a component at a lower frequency, Morse will skip call to
``default_action`` to match desired frequency, as specified in the builder
script (using
:py:meth:`morse.builder.abstractcomponent.AbstractComponent.frequency`).
Internally, The execution frequency of the sensor | actuator | robot  can be
retrieved using the property :py:meth:`morse.core.object.Object.frequency`.

Time management related settings
--------------------------------

At the moment, there are two strategies for handling time at the Morse level:

- the **Best effort** strategy try to handle your requirement in "real-time",
  i.e. one second of time in simulation is equal to one second of time in the
  real world. To do that, Morse may need to drop some frames. Simulator may be
  less accurate too. The "good" point is that you do not need to care too much
  about simulated-time in your tested software. It is the default mode.

- the **Fixed Simulation Step** strategy handles all physical / logical steps,
  at fixed simulation step. It means that between each step, the simulation
  acts as if '1 / base_frequency' sec has elapsed. The simulation is so more
  accurate, but simulated time may diverge from real time. In the previous
  pseudo-code, in this mode ``n`` is always equal to 1, so you don't care
  about ``logic_step_max`` and ``physics_step_max``.

These different strategies are implemented in :py:mod:`morse.core.morse_time`.

The used strategy is selected at the builder level, through the method
:py:meth:`morse.builder.environment.Environment.set_time_strategy`.

Since 1.3, it is also possible to synchronise the simulation step, and so the
time with an external clock. This synchronisation mechanism makes sense in
several scenario:

- in **Fixed Simulation Step**, to synchronise with a ``logic world`` clock
  (or a ``real-time clock`` if it is fast enough)
- if you don't care of the physical engine at all
- in **Best effort**, with ``base_frequency`` > 60.0, to make the simulator more
  periodic. Indeed, as shown in the pseudo-code, the logic / physics
  inner-loop is called as fast as possible. You need an external tool to
  synchronise the inner-loop. See `this issue <https://github.com/morse-simulator/morse/issues/683>`_ 
  for a longer discussion of the subject.

For the moment, this mechanism is implemented for two middlewares (search for
the **time_sync** parameter):

- :doc:`../user/middlewares/socket`
- :doc:`../user/middlewares/hla`

Morse 1.4 introduces the tool **morse_sync**, allowing to send periodically a
signal to the synchronisation socket. It is usable in an "automatic way",
using the method
:py:meth:`morse.builder.environment.Environment.use_internal_syncer`.

Since 1.4 (and Blender > 2.77), it is possible to accelerate / slowdown the
simulation time. At the builder level, it is available through the method
:py:meth:`morse.builder.environment.Environment.set_time_scale`. It is also
possible to change it dynamically using the **time** service `set_time_scale`.

Default settings
----------------

Since Morse 1.4, Morse tries to compute the best settings for your simulation.
It is controllable by the flag ``time_auto_tune`` from
:py:meth:`morse.builder.environment.Environment`. The default
settings are the following:

- Best Effort
- base_frequency is selected according to the faster component specified in
  the builder script
- v-sync is disabled


Accessing time
--------------

In the simulator itself, you can access to the simulated time via
:py:data:`morse.core.blenderapi.persistantstorage().time.time`. It returns the
simulated time as the number of seconds (in float) since Epoch, as done by
:py:meth:`time.time`. More precisely, at startup, the simulated is initialized
with :py:meth:`time.time` and then progress depending of the selected
strategy. The precision depends of the underlaying implementation of
:py:meth:`time.time` and the speed of simulation. If you run a simulation at
60 Hz, the simulator clock will be updated about every 15 ms.

Moreover, in a lot of situations, you do not want to access the
simulated time directly, but at the time as seen by the current robot. To do that, you
must call the method :py:meth:`morse.core.robot.Robot.gettime`. It allows
different modifiers to be added for different robots, triggering all the nice temporal
issues you must address in multi-robot situations. The
:doc:`../user/sensors/clock` exposes the time, as seen by a specific
robot.

Last, a set of services in :py:mod:`morse.services.time_services` allows to
retrieve the simulated time and various statistics about it.
 
