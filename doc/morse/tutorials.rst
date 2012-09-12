Tutorials 
=========

Welcome in the MORSE tutorials section!

As a academic simulator for robotics, MORSE is a large piece of software which may require some time to master.

However, you should hopefully be able to get very quickly your first simulation running.

What is the general workflow?
-----------------------------

MORSE relies on the Blender 3D package to model and run your simulation.

While it is possible to create and configure a simulation entierly from the
:doc:`Blender interface<user/advanced_tutorials/editing_in_blender>`, it is easier
and ways faster to rely on the so-called **Builder** scripts: these scripts
are simple Python scripts that describe simulations. More about that in a
second.

Once you have a description of your simulation (as a Builder script or as
a Blender file), you can run it::

 $ morse run <your file>

Now, instruct your :doc:`middleware (ROS, YARP,...)<user/integration>` to address
the simulator instead of the real robot, and start your control softwares as
you would do on a real robot. If you are not relying on an explicit middleware,
you can also use the :doc:`socket interface <user/middlewares/socket>` :tag:`sockets`
or the :doc:`Python API <pymorse>` :tag:`pymorse`.

.. note::

    In various places in the documentation, you will see labels like
    :tag:`builder`, :tag:`ros` or :tag:`service`. They denote the main subjects
    that the section or the tutorial deal with.

The first simulation
--------------------

First of all, :doc:`install MORSE<user/installation>` if it is not already
done.

Next step, :doc:`user/beginner_tutorials/tutorial`. :tag:`builder`
:tag:`sockets`

Basic MORSE user interface
--------------------------

The commands to launch the simulator and the basic controls are explained in this section.


.. toctree::

    user/basic_morse
    

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
    
Multi-node tutorials
--------------------

These tutorials help setting up a multi-node simulation environment.

.. toctree::
    :glob:
    :maxdepth: 1

    user/multinode/tutorials/*


Blender specific instructions
-----------------------------

Since MORSE is completely built over Blender, it is important to know some of its functionality.

.. toctree::
    :glob:
    :maxdepth: 1

    user/blender_tutorials/*




