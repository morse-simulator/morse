Tutorials 
=========

General Workflow description
----------------------------

This section will describe how to build a complete simulation scenario, from the creation of a custom
robot with predefined sensors and actuators to the complete scene, including
other robots or humans.

.. toctree::

    user/user_workflow
    

Interacting with MORSE
----------------------

Applications have two main ways to interact with the simulator:

1. Using **RPC**-oriented calls, or
2. Using **stream**-oriented interfaces.

RPC calls are typically used to remotely configure the simulator or start
background tasks, while most of data transmissions usually rely on stream-based
interface. Both can be used in MORSE.

Beginners tutorials
-------------------

.. toctree::
    :glob:
    :maxdepth: 1

    user/beginner_tutorials/*

Intermediate tutorials
----------------------

These tutorials provide more in-depth explanations of how to setup simulations with specific requirements.

.. toctree::
    :glob:
    :maxdepth: 1

    user/advanced_tutorials/*

Blender specific instructions
-----------------------------

Since MORSE is completely built over Blender, it is important to know some of its functionality.

.. toctree::
    :glob:
    :maxdepth: 1

    user/blender_tutorials/*




