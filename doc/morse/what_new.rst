What's new in MORSE 1.4?
========================

General
-------

- `Numpy <http://www.numpy.org/>`_ is now needed for Morse. It is used in
  several places where computations using mathutils is not precise enough
  (float vs double precision).
- Time management has been improved in various way. A **morse_sync** tool has
  been introduced to improve precision and timing of high-frequency components 
  (`#683 <https://github.com/morse-simulator/morse/issues/683>`_).
  If available in Blender (Blender > 2.77), it is also possible to accelerate
  or slow-down the simulation (`#388 <https://github.com/morse-simulator/morse/issues/388>`_).
  Moreover, Morse now try to compute automatically the right time settings.
  If you meet any problem related to time, make sure to read the
  :doc:`dev/time_event` and / or report issue to the Morse project.

Components
----------

Robots
++++++

- the :doc:`user/robots/human` in MORSE has been entirely rewritten. The new
  human model is much simpler, yet much nicer (in particular, it features mesh
  skinning for good looking animations).  On the downside, the interactive mode
  is gone for now. Depending on interest, it can be revived in a future version
  (possibly through external scripts, for added flexibility).


Actuators
+++++++++

- the semantic of the :doc:`user/actuators/waypoint` and
  :doc:`user/actuators/destination` actuators has slightly changed: once the
  destination is reached, they do not attempt anymore to actively stay at this
  position. This permits another motion actuator to 'take over' the control of
  the robot. The previous behaviour is still desirable in certain cases (notably
  for flying robots), and can be re-enabled by setting the property
  ``RemainAtDestination`` to true:
  ``motion.properties(RemainAtDestination=True)``. This option is also added to
  the :doc:`user/actuators/rotorcraft_waypoint` actuator, but it defaults to
  true (hence, no behaviour change compared to MORSE 1.3).
- the :doc:`user/actuators/orientation` actuator has been enhanced to possibly
  work more realistically, by limiting the speed of the rotations. The default
  is still to go directly to the desired orientation.
- The :doc:`user/actuators/keyboard` and :doc:`user/actuators/joystick`
  actuators do not call anymore the robot ``apply_speed`` method with values set
  to zero when no input is received. The previous behaviour prevented them to be
  used in combination with another motion actuator (they would always overwrite
  other motion commands with zeros).
- The :doc:`user/actuators/armature` actuator has two new services
  (``rotate_joints`` and ``translate_joints``) that let the user set the
  rotations/translations of only a subset of the armature's joints by providing
  a custom mapping ``{joint name: value}``.
- The :doc:`user/actuators/rotorcraft_attitude` has been extended to be able
  to control the rotorcraft in yaw rate or in absolute yaw (using the
  ``YawRateControl`` property). If it is the case, you can configure if you
  want to configure to use ``normal yaw`` or ``north`` using the
  property ``UseAngleAgainstNorth``. Last, you can configure the actuator to
  use a linear or quadratic thrust model using ``LinearThrust``.
- Introduce the :doc:`user/actuators/drag` "actuator" which allows to simulate
  the drag (air resistance) force opposite to the move of the robot. It allows
  more realistic simulation (if desired).
- Introduce the :doc:`user/actuators/external_force` actuator which allows to 
  apply external force (typically force from the environment such as wind) to
  a robot. It has the same interface than :doc:`user/actuators/force_torque`,
  but apply force in the global frame.
- Introduce the :doc:`user/actuators/quadrotor_dynamic_control` actuator which
  allows to control a quadrotor from the speed of its four engine, using
  simple dynamic equation.


Sensors
+++++++

- **longitude**, **latitude** and **altitude** are not anymore properties of
  :doc:`user/sensors/gps` but must be set at the environment level. Moreover,
  the property **angle_against_north** allows to configure the angle between
  the X-axis and the geographic north (must be positive when the Blender
  X-axis is East of true North, negative if is West of true North).
- Introduce the new high-level sensor :doc:`user/sensors/attitude`, allowing
  to compute the attitude of the system
- Introduce the sensor :doc:`user/sensors/magnetometer`, which allows to
  compute the magnetic field vector of the Earth.
- Extend the sensor :doc:`user/sensors/imu`, to return also the magnetic field
  vector.
- Fixed the :doc:`user/sensors/collision` sensor: it now detects collision only
  when it is actually colliding (before, any object in a 1x1x1m box around the
  sensor would return a collision). While here, improve the documentation with a
  complete example.
- Introduce the sensor :doc:`user/sensors/airspeed`, which allows to compute
  the speed of a vehicle relative to the air.

Modifiers
+++++++++

- Introduce ECEF, Geodetic, Geocentric modifiers, allowing to convert
  coordinates from Blender world to ECEF-r or Geodetic or Geocentric
  coordinates (and vice-versa). It should improve interoperability with flight
  systems in general.
- Introduce Feet modifier, to convert imperial units to meter buts (and
  vice-versa)

Middlewares
-----------

General
+++++++

- Introduce a binding for the `Mavlink protocol
  <http://qgroundcontrol.org/mavlink/start>`_, easing the interoperability of
  Morse with a lot of free autopilots / architectures.

ROS
+++

- Some ROS 'housekeeping' has been performed in this release, including
  removing the need for ``rospkg`` (easier installation!), removing ROS interface
  with the non-standard (and unused?) ``JointPositions`` message and removing
  references ``roslib.load_manifest()``, a memory of rosbuild-era.

HLA
+++

- Handle automatically the case where attributed published by Morse are not
  owned by it.
- Allow to specify a ``stop_time`` for the simulation (in simulated seconds)
- Make ``lookahead`` configurable for the Morse federate

YARP
++++

- Add an adapter for :doc:`user/sensors/depth_camera`.

Builder API
-----------

API addition
++++++++++++

- It is now possible to import environment composed of multiples scenes. The
  user should select which is the ``main_scene`` when importing the environment.
  Moreover, a method ``Environment.set_background_scene`` has been added to configure the
  scene to use in background (`#651 <https://github.com/morse-simulator/morse/issues/651>`_).
- The method ``bpymorse.set_speed``, used to changed the frequency of Morse
  main loop is now deprecated in favor of ``Environment.simulator_frequency``.
- The method ``Environmement.set_time_scale`` allows to accelerate or
  slow-down the simulation (`#388 <https://github.com/morse-simulator/morse/issues/388>`_).
- The new method ``Environment.use_vsync`` allows to control the vsync
  parameter


Pymorse
-------

- Robots created in loop are handled smartly. They are still usable as
  previously, but it is also possible to access them using the list foos (if
  your robot name is foo) (`#358  <https://github.com/morse-simulator/morse/issues/358>`_).
- Streams are now created lazily, fixing control with large number of robots /
  sensors (`#626  <https://github.com/morse-simulator/morse/issues/626>`_).

Previous releases
-----------------

.. toctree::
    :maxdepth: 1	

    releasenotes/1.3
    releasenotes/1.2
    releasenotes/1.1
    releasenotes/1.0
    releasenotes/0.6
    releasenotes/0.5
    releasenotes/0.4
