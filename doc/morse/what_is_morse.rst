What is MORSE?
==============

.. image:: ../media/simu_render_indoors.jpg
   :width: 300
   :align: center
.. Introducing MORSE

MORSE is a **generic simulator for academic robotics**. It focuses on realistic
3D simulation of **small to large environments, indoor or outdoor**, with **one
to tens of autonomous robots**.

MORSE can be entirely controlled from the **command-line**. Simulation scenes are
**generated from simple Python scripts**.

MORSE comes with a set of :doc:`standard sensors <components_library>` (cameras,
laser scanner, GPS, odometers, etc.), :doc:`actuators <components_library>` (speed
controllers, high-level waypoints controllers, generic joint controllers), and
:doc:`robotic bases <components_library>` (quadrotors, ATRV, Pioneer3DX, generic
4 wheel vehicle, PR2, etc.). New ones can easily be added.

MORSE rendering is based on the `Blender Game Engine
<http://www.blender.org>`_.  The OpenGL-based Game Engine supports shaders,
provides advanced lightning options, supports multi-texturing, and uses the
state-of-the-art `Bullet <http://bulletphysics.org>`_ library for physics
simulation.

Simulation with MORSE
---------------------

In MORSE, *simulations* are :doc:`small Python scripts <user/builder>` that
describe the robots and the environment. MORSE provides several command-line
tools to create stubs, and it takes virtually no time to get a first simulation
up and running.

.. image:: ../media/caylus.jpg
   :width: 400
   :align: center
.. MORSE used for simulation of ground-air multi-robot cooperation

One of the MORSE's main design choices is to provide the ability to
select a simulation's *degree of realism*: if you are working on vision,
you need accurate camera sensors, but may not care about the realism of
your motion controller, and you may find a waypoint controller good
enough and easier to use. On the other hand, if you work on robot
supervision, you may prefer to skip the perception stack and directly
work with object IDs and positions. MORSE lets you define how realistic
the different components of you robot need to be, to meet your needs.

MORSE also supports two different strategies for handling time: **best effort**,
that tries to keep a real-time pace, at the cost of dropping frames if
necessary, or **fixed step** that ensures that the simulation is accurate. In this
case, MORSE exports its own clock that can be used to synchronize with other
time-dependent modules in your system.

Extending MORSE
+++++++++++++++

MORSE is mostly written in Python: except for computationally intensive processing
(like 3D rendering or physics simulation), MORSE is a purely Python
application. This enables easy and fast modification of the source code.

.. image:: ../media/python-powered.png
   :align: center
.. MORSE extensively uses Python

In addition, MORSE has been designed to be modular: adding a new sensor, a new
actuator, a post-processing function (like applying a noise function), adding new
services, or even a completely new communication middleware is reasonably easy and
documented.


Integration in your workflow
----------------------------

MORSE does not make any assumptions about your architecture. MORSE currently
supports **6 open-source middlewares (ROS, YARP, Pocolibs, MOOS, HLA and
Mavlink)**. :doc:`Check here for the exact list of features supported for each
middleware <user/integration>`.

It also supports a **simple socket-based protocol** for easy integration with
other languages/toolboxes. **Complete bindings for Python** are provided.

MORSE comes with a :doc:`set of standard sensors and actuators
<components_library>`. To suit your specific needs, MORSE also provides a
:doc:`lightweight overlay <user/overlays>` mechanism to quickly change the name
and types of exchanged data flows.

Also note that MORSE benefits from Blender's import/export capabilities: existing
models in many 3D formats (Collada, DXF, 3DS Max, VRML, etc.) can be
used to build robots and environments.


Performance
-----------

MORSE is able to handle dozens of robots in a single environment as long as
cameras are not simulated (because of bandwidth limitations).

For instance, MORSE running on an Opteron quadcore 2GHz, in :doc:`headless mode <headless>` (i.e.
**without 3D acceleration**), can simulate:

- one robot with a pose sensor at 250Hz
- 50 robots with pose sensors at ~90Hz
- 10 robots with a pose sensor at ~40Hz and a laser scanner scanning at ~18Hz

(These were measured with standard ROS tools.)

When cameras do no need to be simulated, MORSE offers a **fast mode** with greatly
improved performance.

MORSE is also suitable for large simulations of complex robots: MORSE can be run
as a distributed network of :doc:`simulation nodes <multinode>`. Each node
automatically synchronizes with the others (however, due to network latencies, don't
expect to simulate accurate physical interactions in the distributed mode).

.. image:: ../media/ocean.jpg
   :width: 300
   :align: center
.. Multi-robot simulation: one helicopter cooperates with a submarine
   for mine hunting.

MORSE installation
------------------

MORSE is packaged in Debian/Ubuntu: `sudo apt-get install morse-simulator`

MORSE is also easy to compile from source. Its only dependencies are
Python, Blender, and the middlewares you want to use. Any
Linux distribution should provide all of the required dependencies out of the box.

:doc:`MORSE installation <user/installation>` is based on CMake, and allows you
to install support for only those parts relevant to your needs (e.g.,
you don't have to install YARP if you are using ROS, and so on).

MORSE is also available as a `robotpkg <http://robotpkg.openrobots.org>`_
package: ``robotpkg`` is a package manager for robotics related software that
will automatically take care of all of MORSE's dependencies.


MORSE as a software project
---------------------------

.. image:: ../media/osi-license.png
   :align: center
.. MORSE is an open-source project

MORSE and all the libraries it relies on are open-source projects.

MORSE itself is licensed under a permissive BSD license: you can use it for any
purpose, without having to share your modifications back.

This also means that MORSE follows an open development process: you can fork
MORSE source code on `GitHub <http://github.com/morse-simulator/morse>`_ and
everybody is invited to propose new features, report bugs, and submit patches.

MORSE tries to follow software development good practises, like `continuous
<https://travis-ci.org/morse-simulator/morse>`_ `integration
<http://www.openrobots.org/morse/doc/latest/contributing.html#build-status>`_.

Community
+++++++++

According to `Ohloh <https://www.ohloh.net/p/morse_simulation_engine>`_, MORSE
is an active and mature project, with well over 20 contributors.

MORSE is used by over 15 robotic labs in the world, and questions on its
mailing-lists (`morse-users@laas.fr
<https://sympa.laas.fr/sympa/subscribe/morse-users>`_ and `morse-dev@laas.fr
<https://sympa.laas.fr/sympa/subscribe/morse-dev>`_) are usually answered within
a few hours.

MORSE is also based on `Blender <http://www.blender.org>`_ for modelling, 3D
rendering with shader support, import/export of 3D models, and `Bullet
<http://bulletphysics.org>`_ for physics simulation.

These two huge open-source projects are very active and are supported by large
communities of users and developers.

This means that there are lots of tutorials, code examples, reusable snippets, etc.

This also ensures that, even if the MORSE core team disappeared, you would
still be able to ask for support!

MORSE also integrates with other large open-source projects like `ROS
<http://www.ros.org>`_, which further anchors it into the open-source robotics
community.

Documentation
+++++++++++++

MORSE has complete and up-to-date online documentation, both for users and
developers: `MORSE documentation <http://www.openrobots.org/morse/doc>`_.

Several :doc:`tutorials <tutorials>` are also available, for quickly
getting started.

.. image:: ../media/documentation.jpg
   :width: 500
   :align: center
.. MORSE documentation


Focus on academic requirements
-------------------------------

MORSE was created at `LAAS-CNRS <http://www.laas.fr>`_, a French public laboratory,
one of the biggest in robotics.

`Many more universities and institutes
<https://github.com/morse-simulator/morse/blob/master/doc/survey/first-survey/report.tex>`_ have joined the effort and collaboratively take part in assuring the
future of MORSE.

Our close collaborations with academic research in robotics worldwide ensures
that many new innovations are quite quickly added to our roadmap.

Check here :doc:`MORSE related publications and workshop <media>`.

Other Features
--------------

Some additional features provided by MORSE are:

- human-robot interaction simulation, with a controllable human avatar
- deep integration with unit-testing frameworks: use MORSE to test your own
  software

.. image:: ../media/hri.jpg
   :width: 300
   :align: center
.. MORSE used in a human-robot interaction scenario


MORSE limitations
-----------------

Last but not least, MORSE has some important limitations you must be aware of
when assessing simulation solutions:

- MORSE has (almost) no graphical user interface. While some consider this to be an
  advantage, others may miss it. An important correlate: MORSE is primarily
  targeted at experienced computer scientists. While we spend a lot of time
  designing a convenient and intuitive interface (after all, we use it on a
  daily basis for our own research!), to use MORSE you need to be
  comfortable using command-line tools.

- unlike some other simulators, MORSE does not include any advanced algorithms
  (like path planning). You are expected to run provide them yourself in
  your own robot software stack.

- we do not (yet?) consider MORSE to be a physically accurate simulator: while we
  rely on a state-of-the-art physics engine (Bullet), do not expect to
  accurately simulate robot arm dynamics or fine grasping. Other projects are
  doing this much better (e.g., `OpenGrasp <http://opengrasp.sourceforge.net/>`_
  for grasping).

- MORSE is mostly developed and supported on Linux. MORSE is known to run
  on MacOSX and Microsoft Windows, but only limited support can be provided for
  these platforms.

- As a not-for-profit, academic project, we do not offer any professional
  support beyond the documentation and the public mailing-lists. However,
  there's nothing to stop third party companies from providing commercial
  services based on MORSE.
