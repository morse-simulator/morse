MORSE, the OpenRobot Simulator
==============================

+------------------------------------------------------------------------------+
|              .. image:: ../media/openrobots-simulator.png                    |
+--------------------------------------+---------------------------------------+
| .. image:: ../media/stae_logo.png    | .. image:: ../media/blender_logo.png  | 
|    :width: 200                       |    :width: 200                        |
+--------------------------------------+---------------------------------------+
| .. image:: ../media/laas.png         | .. image:: ../media/onera.png         |
|    :width: 200                       |    :width: 200                        |
+--------------------------------------+---------------------------------------+

Contents:

.. toctree::
   :maxdepth: 2
   
   user/summary.rst
   dev/summary.rst
   
Introducing MORSE
-----------------

This software is partially financed by the `RTRA STAE <http://www.fondation-stae.net>`_,
as part of the `ROSACE <http://www.fondation-stae.net/fr/actions/projets-cours.html>`_ project, and by the DGA founded  `PEA ACTION <http://action.onera.fr>`_.

.. image:: ../media/simu_render_indoors.jpg
   :width: 300
.. Introducing MORSE

Main features
-------------

- A versatile simulator for **generic mobile robots simulation** (single or 
  multi robots),
- Enabling **realistic** and **dynamic** environments (with other interacting 
  agents -humans- or objects), 
- Don't reinvent the wheel: critical components reused from other opensource 
  projects (**Blender** for 3D rendering + physical simulation + UI, dedicated 
  robotic middlewares for communications + robot hardware support),
- **Seamless workflow**: since the simulator rely on Blender for both modelling and the realtime 3D engine, creating and modifying a simulated scene is straigthforward.
- Entierely scriptable in **Python**,
- Adaptable to various **level of simulation realism** (for instance, we may want to simulate exteroceptive sensors like cameras in certain cases and access directly to a higher level representation of the world -like labelled artifacts- in other cases),
- Currently compatible with **YARP** and LAAS OpenRobots robotics frameworks,
- Fully open source, BSD-compatible.

Available sensors and actuators
-------------------------------

MORSE has been designed to be modular and explicit interfaces ease the addition of new features (sensors, actuators, communication middleware, etc.).

The first release of MORSE features the following set of sensors and actuators.

Sensors
+++++++
- Mono-camera, stereo-camera, 3D camera (depth of field), semantic camera (visible labelled objects)
- 2D laser range finder
- GPS
- Gyroscope
- Thermometer

Controllers
+++++++++++
- (v, ω) servoing
- waypoints
- manual (keyboard/mouse)

Communication middleware
++++++++++++++++++++++++
- YARP
- pocolibs (shared memory)

Documentation
-------------

- :doc:`User's documentation <user/summary>`
- :doc:`Developers's documentation <dev/summary>`

Media
-----

Outdoors scenes
+++++++++++++++

+------------------------------------------+------------------------------------------+
| .. figure:: ../media/outdoor_example.jpg | .. figure:: ../media/outdoor_example.jpg |
|                                          |                                          |
|    An ATRV in an outdoor scenario        |    In this sequence, robots are          |
|                                          |    controlled by waypoints, one of the   |
|                                          |    controllers type offered by MORSE.    |
|                                          |    The video shows as well the output of |
|                                          |    three cameras on YARP channels.       |
+------------------------------------------+------------------------------------------+

.. <object width="400" height="300"><param name="allowfullscreen" value="true" /><param name="allowscriptaccess" value="always" /><param name="movie" value="http://vimeo.com/moogaloop.swf?clip_id=9825826&amp;server=vimeo.com&amp;show_title=1&amp;show_byline=1&amp;show_portrait=0&amp;color=&amp;fullscreen=1&amp;group_id=" /><embed src="http://vimeo.com/moogaloop.swf?clip_id=9825826&amp;server=vimeo.com&amp;show_title=1&amp;show_byline=1&amp;show_portrait=0&amp;color=&amp;fullscreen=1&amp;group_id=" type="application/x-shockwave-flash" allowfullscreen="true" allowscriptaccess="always" width="400" height="300"></embed></object>


Indoors scenes
++++++++++++++

+---------------------------------------+--------------------------------------+
| .. figure:: ../media/indoors_sick.jpg | .. figure:: ../media/indoors_sick.jpg|
|                                       |                                      |
|    Real-time simulation of a SICK     |    This video demonstrates real-time |
|    laser range finder in an indoors   |    physics simulation, laser range   |
|    environment.                       |    finder simulation and redirection |
|                                       |    of the robot cameras (on the wall)|
+---------------------------------------+--------------------------------------+

.. <object width="400" height="300"><param name="allowfullscreen" value="true" /><param name="allowscriptaccess" value="always" /><param name="movie" value="http://vimeo.com/moogaloop.swf?clip_id=9825888&amp;server=vimeo.com&amp;show_title=1&amp;show_byline=1&amp;show_portrait=0&amp;color=&amp;fullscreen=1&amp;group_id=" /><embed src="http://vimeo.com/moogaloop.swf?clip_id=9825888&amp;server=vimeo.com&amp;show_title=1&amp;show_byline=1&amp;show_portrait=0&amp;color=&amp;fullscreen=1&amp;group_id=" type="application/x-shockwave-flash" allowfullscreen="true" allowscriptaccess="always" width="400" height="300"></embed></object>

More videos are available `online on Vimeo <http://vimeo.com/groups/blenderandrobotics>`_.

On the road-map
---------------

The first release of MORSE contains only a subset of the final simulator specification.

Amongst the planned features:

- Support for arms simulation, based on inverse kinematics. This has been separately developped by the Leuven's university and will be merge into MORSE over the next releases,
- Raw sockets interface + full compatiblity with the ROS robotics framework (other robotics framework are planned as well. Let us know if you want to contribute in this area),
- Developement of the user interface,
- Scalablity (both in term of simulation capacity and ease of deployment),
- Multi-node simulations (several Blender nodes can be started on several computer and automaticaly synchronise, which should allow simulations of tenth of robots in the same scene),
- Dedicated supervision node that would allow to: observe the simulation, display logs and metrics, start/stop robots, dynamically alter the scene (like moving an obstacle in front of a robot, etc.).
