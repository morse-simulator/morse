Tutorials 
=========

The MORSE Workflow 
------------------

How to build a complete simulation scenario, from the creation of a custom
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
    :maxdepth: 1

    user/tutorial
    user/advanced_tutorials/editing_in_blender

Intermediate tutorials
----------------------

These tutorials provide more in-depth explanations of how to setup simulations with specific requirements.

.. toctree::
    :glob:
    :maxdepth: 1

    user/advanced_tutorials/*
