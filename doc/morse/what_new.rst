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
  output (#330). 

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

Builder API
-----------

It is now possible to handle loop in builder script (#357). See
:doc:`user/builder` for documentation about it.

API addition
++++++++++++

- `make_ghost` method allows to transform the robot in ``ghost mode``
  (transparent and with no associated physics)
- `mass` method allows to setup the mass of any component.
- `set_log_level` allows to configure easily the level of log of each
  component. A service with similar name allows to do that at runtime too.
  (#337).
- `set_speed_camera` allows to change the speed of the viewport camera.
- `set_friction` allows to change friction parameter with the ground

API changes
+++++++++++

- `add_default_interface` has now a smarter behaviour (#399)

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

What's new in MORSE 1.0?
========================

General
-------

- MORSE is mature! stable release !
- Unit-test coverage has been substantially improved
- Documentation of component is now partially auto-generated: this should bring
  a better, up-to-date, complete documentation, including code examples
- MORSE has a new logo!

Components
----------

- All components now explicitly expose their data fields and properties with
  respectively `add_data` and `add_property`.
- Introduced "abstraction levels" that allow to define several levels of
  realism for a given component (#166). Many components remain to be ported to
  use this interface, though.
- Many component classes have been renamed to be more readable and match
  Builder conventions
- Creation and configuration of armatures in Blender, suitable for MORSE is now
  properly documented.
- Most of the component do not need a Blender file anymore (only the one with
  complex meshes or armature are kept) (#221).
- Blender file for components do not require any game property of logic brick
  anymore. It is now fully created within the Builder API. This means that any
  Blender model can be used as MORSE model, without specific configuration
  (#241).
- components can now be easily profiled for performance assessment from the
  Builder API.

Robot
+++++

- Substantial improvement regarding the PR2 robot support. Besides 2D
  navigation, the robot's joint state and joint control via standard tools like
  `pr2_tuck_arm` works out-of-the-box. PR2 joint name have been updated to
  match the latest version. Several scripts allow to create a PR2 with variable
  level of equipment.
- Fixed the `WheeledRobot` class of robot that had a erratic physics behaviour.
  Make `Pioneer3DX` inherit from this robot class (#245).

Actuators
+++++++++

- Complete rewrite of the armature actuator. It can now track joints state
  (interpolating joint rotation if required, and not only 'jumping' to the
  target position) and execute trajectories with interpolation. It also
  introduces support for prismatic joints (#231, #232).
- `Waypoint` actuator: improve handling of interruption (the robot motion now
  actually stops).

Sensors
+++++++

- New sensor: depth camera (Thanks to Gilberto's patch in Blender 2.65), with
  specialization like Kinect (#122, #123, #138). It uses Python 3.3
  `memory_view` for fast, copy-less transfer of binary data between the OpenGL
  buffers, the C processor, and the interface.
- Laser sensors have been reorganized and grouped in one single category
  (#155, #226).
- Odometry now expose several abstraction levels (*raw*, *differential*,
  *integrated*)
- New sensor: armature pose. This sensor superseeds previous sensors like
  `kuka_pose` or `pr2_posture` by proving a clean interface to armature states.
- New sensor: Velodyne
- New special *compound sensor* that allows to merge the output of several
  sensors. Used for instance to merge the joints values of the different PR2
  armature in a single joint state (#240).
- Former `rosace` sensor has been renamed to a more appropriate
  `search_and_rescue` sensor.
- Laser scanner ranges ordering has been reversed to match ROS conventions.

Builder API
-----------

- One class per component: for instance, `Robot('atrv')` becomes `ATRV()`. The
  documentation page of each component gives an example.
- New `FakeRobot()` for clock and other static components (like environment
  cameras).
- Former functions `configure_mw` and `configure_service` replaced by new
  `add_interface`, `add_datastream`, `add_service` or `add_default_interface`
  that sets an interface for a whole robot (#217).
- Components are now automatically renamed after the variable names used in the
  Builder script, provide much more natural naming schemes. Names can still be
  configured explicitly with `component.name` (#133).
- Component profiling with `component.profile()`
- Errors in Builder scripts are now better handled, with meaningful error
  messages.
- The simulation can now be configured from the Builder script for 3D output
  (split screen), including configuration of eye separation.
- Added ability to automatically save your scene as a Blender file from the
  Builder script.
- Added a `fastmode` option when setting up the environment: in *fastmode*,
  only wireframes are displayed. This improves MORSE loading time and
  performance, but some sensors (like cameras) won't work. Most of the
  unit-tests now use this mode.

.. warning::

  It basically means that scene are incompatible between release 0.6 and 1.0.
  To help the conversion, you can use the tool available `here
  <https://raw.github.com/morse-simulator/morse/master/tools/convert_0.6_to_1.0.sh>`_
  Basically, you can use it like that::

		sh convert_0.6_to_1.0.sh your_scene.py > your_scene_1.0.py

  You may need to edit the resulting file manually to fix last issues. If you
  get in trouble, feel free to send us a mail on morse-users@laas.fr with your
  scene.

Assets
------

- Added a new `empty` environment, especially suited for tests.

Interfaces
----------

- Interfaces can now implement data serialization/deserialization in explicit
  classes: no more hacky appending of free functions (#144, #145).

ROS
+++

- GPS : cleaned, to be validated
- Odometry now publish both Odometry and TF
- Pose publish only Pose (no more Odometry)
- Laser scanners can now export point clouds (`PointCloud2`)
- New `PointCloud2` publisher for depth camera, Kinect in progress
- Support for the JointTrajectory ROS action for armature control
- Special unit-test class for ROS tests that takes care of setting up an
  appropriate ROS environment (including launching `roscore`).


Sockets
+++++++

- Support for cancelling asynchronous requests
- Support for exporting matrices and 3D transformations

pocolibs
++++++++

- Large rewrite of pocolibs interface, now using `ctypes` instead of SWIG
  bindings. This simplifies a lot the compilation and maintenance of these
  interfaces.

Text
++++

- Improved the `text` interface, to allow for instance output as `.csv` files.

pymorse
+++++++

The Python bindings for MORSE have been completely rewritten, now supporting a
modern asynchronous interface (based on Python 3.2 *futures*). It is also
deemed as feature complete: it supports discovery of the simulation components,
synchronous/asynchronous service invocation (including service cancellation)
and synchronous/asynchronous read/write of datastream (#216).

MORSE unit-tests now use this new API.

Internals
---------

- Substantial changes in MORSE internals:

  - lots of refactoring, to improve code consistency (including
    {middleware->datastream} (#186))
  - many files have been renamed for consistency

Misc
----

- MORSE now uses the MORSE_RESOURCE_PATH environment variable to look after
  custom location for assets: convenient to store your own model out of MORSE
  tree (#187).
- Added configuration file required by the Travis buildbot
- Several large examples or tutorials have been removed (because either
  deprecation or doubtful usefulness)
- New CSS for documentation, based on GitHub *minimal* style.
- Numerous bugfixes, including:

  - the 'objects flying around' bug, that was due to the way Blender handle
    transformation matrices (#139).
  - a bug affecting the color of some materials
  - bug with logging when restarting the simulation in special cases (#183)


Previous releases
-----------------

.. toctree::
	:maxdepth: 1	

	releasenotes/0.6
	releasenotes/0.5
	releasenotes/0.4
