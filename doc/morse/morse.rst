Introducing the Modular OpenRobots Simulation Engine 
====================================================

Welcome to the official documentation for the MORSE project.

Quick start
-----------

.. toctree::
    :maxdepth: 1

    user/installation
    user/basic_morse
    user/tutorial

For any questions about the use of MORSE or if you have any issues with the
simulator, you can drop a mail to the `morse-users@laas.fr <mailto:morse-users@laas.fr>`_ 
mailing-list. You can suscribe to the mailing-list
`here <https://sympa.laas.fr/sympa/subscribe/morse-users>`_.

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
- Currently compatible with **YARP** and LAAS OpenRobots robotics frameworks,
- Fully open source, BSD-compatible.
  
.. image:: ../media/morse_interface.jpg
   :width: 300
   :align: center
.. MORSE interface

MORSE is partially funded by the `Fondation RTRA <htpp://www.fondation-stae.net>`_ within the `ROSACE project <http://homepages.laas.fr/khalil/ROSACE>`_ framework.

.. image:: ../media/stae_logo.png
   :align: center                
   :width: 200

.. image:: ../media/rosace.png
   :align: center
   :width: 200
                   

Installation
------------

.. toctree::
    :maxdepth: 2
    
    user/installation

The MORSE Workflow 
------------------

How to build a complete simulation scenario, from the creation of a custom
robot with predefined sensors and actuators to the complete scene, including
other robots or humans.

.. toctree::

    user/user_workflow

Components library
------------------

.. image:: ../media/morse_robot.jpg
   :width: 300
   :align: center
.. The MORSE robots

MORSE offers an extended set of predefined sensors and controllers that cover 
reasonably well common simulation needs in robotics. It offers also some 
complete robots.

The following page lists all the currently existing components and their
properties: 

.. toctree::
    :maxdepth: 3

    user/component_library

MORSE has also a mechanism to alter input or output data (like adding noise to
a GPS position) by so called *modifiers*: 

.. toctree::
    :maxdepth: 3

    user/modifier_introduction

To learn how to add new components (sensors, robots...), please refer to the 
:doc:`developer documentation <dev/summary>`.

Supported middlewares
---------------------

MORSE relies on *middlewares* to integrate in your robotic architecture.

We currently support only `YARP <http://eris.liralab.it/yarp/>`_, 
`pocolibs <https://softs.laas.fr/openrobots/wiki/pocolibs>`_ and a simple 
text-based socket protocol. More middlewares are expected to be added in the 
next versions (partial `ROS <http://www.ros.org>`_ support is available in 
the development trunk).

Detailled information: 

.. toctree::
    :maxdepth: 3

    user/hooks
    user/supported_middlewares

Tutorials 
---------

Beginners
+++++++++

.. toctree::
    :maxdepth: 1
    
    user/tutorial

Intermediate
++++++++++++

These tutorials provide more in-depth explanations of how to setup simulations with specific requirements.

.. toctree::
    :glob:
    :maxdepth: 1

    user/advanced_tutorials/*

Contributing to MORSE
---------------------

As an open-source project driven by the research community, your contributions are very welcome!

Check the :doc:`Developers documentation <dev/summary>`.

.. toctree::
    :maxdepth: 1

    dev/dev_overview
    dev/dev_workflow
    dev/adding_component
    dev/adding_modifier
    dev/new_middleware

Tips and how-tos 
----------------

.. toctree::
    :glob:
    :maxdepth: 1

    user/tips/*

Media
-----

Publications
++++++++++++

- `Modular Open Robots Simulation Engine: MORSE <http://homepages.laas.fr/gechever/Documents/paper-icra.pdf>`_, ICRA 2011

Screenshots
+++++++++++

+------------------------------------------+------------------------------------------+
| .. figure:: ../media/outdoor_example.jpg |  .. figure:: ../media/indoors_sick.jpg   | 
|                                          |                                          |
|    An ATRV in an outdoor scenario.       |     Real-time simulation of a SICK       |
|                                          |     laser range finder in an indoors     |
|                                          |     environment.                         |
+------------------------------------------+------------------------------------------+
| .. figure:: ../media/hri.jpg             |  .. figure:: ../media/morse_interface.jpg| 
|    :width: 422                           |     :width: 422                          |
|                                          |                                          |
|    Simulation of human-robot             |     The MORSE interface (crude Blender   |
|    interaction: the robot tracks the     |     :-) )                                |
|    posture of the human.                 |                                          |
+------------------------------------------+------------------------------------------+


Videos are also available on the `Blender for Robotics Vimeo group <http://vimeo.com/groups/blenderandrobotics>`_.

On the road-map
---------------

The first release of MORSE contains only a subset of the final simulator specification.

Amongst the planned features for future MORSE releases:

- full compatiblity with the ROS robotics framework (other robotics framework are planned as well. Let us know if you want to contribute in this area),
- support for point cloud sensors (stereo-vision, Velodyne, Kinect,...)
- complete support of the Willow Garage's PR-2 robot, along with all the sensors
- Developement of the user interface,
- Scalablity (both in term of simulation capacity and ease of deployment),
- Multi-node simulations (several Blender nodes can be started on several computer and automaticaly synchronise, which should allow simulations of tenth of robots in the same scene),
- Dedicated supervision node that would allow to: observe the simulation, display logs and metrics, start/stop robots, dynamically alter the scene (like moving an obstacle in front of a robot, etc.).

