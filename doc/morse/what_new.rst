What's new in MORSE 1.2?
========================

General
-------

- Time management in Morse has been clarified (#388). See
  :doc:`time management documentation <../dev/time_event>` for more details about it.

Components
----------

Sensors
+++++++

- Each sensor has now an additional field ``timestamp`` exporting when the
  data has been computed, in simulated time.

Middlewares
+++++++++++

Pocolibs
$$$$$$$$

- ``pocolibs`` is now able to export :doc:`velodyne <user/sensors/depth_camera>`
  sensor.


What's new in MORSE 1.1?
========================

General
-------

- morse tools has now some options to easily create custom simulations: check
  'morse {create|rm|add} --help' for more on that topic! Or have a look to the
  new :doc:`quickstart` tutorial.

Components
----------

- Modifiers use now a class-base scheme, similarly to datastream input /
  output (`#330 <https://github.com/morse-simulator/morse/issues/330>`_).

Actuators
+++++++++

- Improve the :doc:`user/actuators/light` actuator, including more
  configurable parameters (in particular, the color of light and its energy).
  A new service 'toggle' allows to control it.
- New 3D :doc:`user/actuators/sound` actuator {play,pause,stop} from local mp3 file.
- Add a :doc:`joystick <user/actuators/joystick>` actuator allowing to control directly
  your robot using a joystick.


Robots
++++++

- Introduce :doc:`Morsy <user/robots/morsy>`, the Morse mascot, now available directly
  in the simulator.
- Introduce :doc:`PatrolBot </user/robots/patrolbot>`, a differential robot
  developed by MobileRobots.

Sensors
+++++++

- A new sensor :doc:`user/sensors/velocity` allows to retrieve properly the
  velocities of a robot which use a physic controller.
- The :doc:`user/sensors/semantic_camera` performance has been improved. It
  has a new option 'noocclusion' to disable occlusion testing and get even
  better performances.
- The :doc:`user/sensors/thermometer` has been reworked to handle multiple
  fire sources, potentially of different nature
- Introduce a new sensor :doc:`user/sensors/collision` which allows to detect
  if the robot is in collision with some objects of the environment.
- Fix Camera resolutions ratio issue `#371
  <https://github.com/morse-simulator/morse/issues/371>`_. We now create one
  Blender scene for each camera in the simulation, with specific render
  resolution.

Builder API
-----------

It is now possible to handle loop in builder script (`#357
<https://github.com/morse-simulator/morse/issues/357>`_). See
:doc:`user/builder` for documentation about it.

API addition
++++++++++++

- `make_ghost` method allows to transform the robot in ``ghost mode``
  (transparent and with no associated physics)
- `mass` method allows to setup the mass of any component.
- `set_log_level` allows to configure easily the level of log of each
  component. A service with similar name allows to do that at runtime too.
  (`#337 <https://github.com/morse-simulator/morse/issues/337>`_).
- `set_speed_camera` allows to change the speed of the viewport camera.
- `set_friction` allows to change friction parameter with the ground

API changes
+++++++++++

- `add_default_interface` has now a smarter behaviour (`#399
  <https://github.com/morse-simulator/morse/issues/399>`_)

Pymorse
-------

The Python bindings for MORSE have been completely rewritten and is now much
more efficient (based on *asynchat* API). However, it is mostly an internal
rewrite, and the interface does not change.

Multi-node
----------

Rewrite the :doc:`user/multinode/socket` client/server (internal). Use JSON
instead of unsafe `pickle`.

Tools
-----

- Remove old Blender 2.4 geolandloader code
- Add tools/terrain/blend_dtm.py to build map from a DEM and an Orthoimage



Previous releases
-----------------

.. toctree::
    :maxdepth: 1	

    releasenotes/1.0
    releasenotes/0.6
    releasenotes/0.5
    releasenotes/0.4
