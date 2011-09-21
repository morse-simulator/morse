Modular OpenRobots Simulation Engine - Documentation 
====================================================

Welcome to the official documentation for the MORSE project.

For any questions about the use of MORSE or if you have any issues with the
simulator, you can drop a mail to the `morse-users@laas.fr <mailto:morse-users@laas.fr>`_ 
mailing-list. You can subscribe to the mailing-list
`here <https://sympa.laas.fr/sympa/subscribe/morse-users>`_.

Bug reports are always welcome on `our bug-tracker <https://softs.laas.fr/bugzilla/enter_bug.cgi?product=morse>`_.



+-------------------------------+--------------------------------------+
|                               |                                      |
|    What's new in MORSE 0.4?   |                                      |
|                               |                                      |
+-------------------------------+--------------------------------------+
|   .. toctree::                |                                      |
|    :maxdepth: 1               |                                      |
|                               |                                      |
|    user/installation          |                                      |
|                               |                                      |
+-------------------------------+--------------------------------------+
|                               |                                      |
|                               |                                      |
|                               |                                      |
+-------------------------------+--------------------------------------+
|                               |                                      |
|                               |                                      |
|                               |                                      |
+-------------------------------+--------------------------------------+
|                               |                                      |
|                               |                                      |
|                               |                                      |
+-------------------------------+--------------------------------------+

.. toctree::
    :maxdepth: 1
    :glob:
    
    *.rst

What is MORSE? 
--------------

.. image:: ../media/simu_render_indoors.jpg
   :width: 300
   :align: center
.. Introducing MORSE

- A versatile simulator for **generic mobile robots simulation** (single or
  multi robots),
- Enabling **realistic** and **dynamic** environments (with other interacting
  agents -humans- or objects), 
- Don't reinvent the wheel: critical components reused from other open source
  projects (**Blender** for 3D rendering + UI, **Bullet** for physics
  simulation, dedicated robotic middlewares for communications + robot hardware
  support),
- **Seamless workflow**: since the simulator rely on Blender for both modeling
  and the real time 3D engine, creating and modifying a simulated scene is
  straightforward.
- Entirely scriptable in **Python**,
- Adaptable to various **level of simulation realism** (for instance, we may
  want to simulate exteroceptive sensors like cameras in certain cases and
  access directly to a higher level representation of the world -like labeled
  artifacts- in other cases),
- Currently compatible with **ROS**, **YARP** and the LAAS OpenRobots robotics
  frameworks, 
- easy to debug and integrate to other environments via a simple socket
  interface,
- Multi-node simulations (several Blender nodes can be started on several
  computer and automatically synchronise, which should allow simulations of
  tenth of robots in the same scene) **EXPERIMENTAL**
- Fully open source, BSD-compatible.
  

  

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

Supported middlewares
+++++++++++++++++++++

MORSE relies on **middlewares** to tightly integrate in your robotic architecture.

We currently support a generic socket-based interface, `YARP
<http://eris.liralab.it/yarp/>`_, `ROS <http://www.ros.org>`_, `pocolibs
<https://softs.laas.fr/openrobots/wiki/pocolibs>`_ and `MOOS
<http://www.robots.ox.ac.uk/~mobile/MOOS/wiki/pmwiki.php>`_. More middlewares may be
added in coming versions. Drop us a mail if you have specific needs.

.. note::
  Some components/services may not be supported by a specific middleware. Please check the :ref:`compatibility-matrix`.

Detailled information: 

.. toctree::
    :maxdepth: 3

    user/hooks
    user/overlays
    user/supported_middlewares

Simulation supervision
++++++++++++++++++++++

Besides component-specific services and data stream (documented on each component's
own documentation page), MORSE provides a set of *supervision services* that
may be used to remotely control the global behaviour of the simulator:

.. toctree::
	:maxdepth: 1

	user/supervision_services

