What's new in MORSE 1.0-beta1?
==============================

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

- Substential changes in MORSE internals:
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
