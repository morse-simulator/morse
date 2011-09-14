The MORSE Component Library
===========================

The modular concept of MORSE is based on individual components with specific
functions that replicate the parts of a real robot. Components in MORSE can
belong to three main types:

- `Bare robotic bases`_
- `Sensors`_
- `Actuators`_

In general, Sensors generate data from the simulated world, to be used by
external programs. Alternatively, Actuators read data from outside Blender to
be applied inside the simulation. Both Sensors and Actuators must be linked to
a Robot to function.

Each component has its own data structure with the variables it requires to function.

The data generated inside Blender can be considered to be "perfect", because it
is very accurate and is not affected by real phenomenons. To make the data
generated more realistic, it is possible to use :doc:`modifiers <modifier_introduction>` that
will alter the data collected by the sensors. The modifiers can add noise,
apply corrections or delays, or change the data with respect to other criteria.

To interact with the outside world, components rely on Middlewares connected to
Blender. See also :doc:`the list of supported middlewares <supported_middlewares>`
for a further explanation.

The following diagram shows the data flow from the Simulation to the sensor,
then the modifiers and finally the middleware to send the data to external
programs:

.. image:: ../../media/component_diagram.png
    :align: center
    :width: 500
.. Component data flow

The data flow is similar for actuators, except that the direction is inverted,
with the data arriving first from the evaluated software via the middleware,
then processed by the modifiers and finally applied in the simulation.


Sensors
-------

.. toctree::
    :glob:
    :maxdepth: 1

    sensors/*


Actuators
---------

.. toctree::
    :glob:
    :maxdepth: 1

    actuators/*

Bare robotic bases
------------------

.. toctree::
    :glob:
    :maxdepth: 1

    robots/*


Other components
----------------

.. toctree::
    :glob:
    :maxdepth: 1

    others/*
