What's new in MORSE 1.4?
========================

General
-------

- `Numpy <http://www.numpy.org/>`_ is now needed for Morse. It is used in
  several places where computations using mathutils is not precise enough
  (e.g., float vs. double precision).
- Time management has been improved in various way. A **morse_sync** tool has
  been introduced to improve the precision and timing of high-frequency components 
  (`#683 <https://github.com/morse-simulator/morse/issues/683>`_).
  If available in Blender (Blender > 2.77), it is also possible to accelerate
  or slow-down the simulation (`#388 <https://github.com/morse-simulator/morse/issues/388>`_).
  Moreover, Morse now tries to automatically compute the correct time settings.
  If you encounter any time related problem, make sure you read
  :doc:`dev/time_event` and / or report the issue to the Morse project.

Components
----------

Robots
++++++

- The MORSE :doc:`user/robots/human` has been entirely rewritten. The new
  human model is much simpler, yet much nicer (in particular, it features mesh
  skinning for attractive animations).  On the downside, the interactive mode
  has gone for now. Depending on interest, it may be revived in a future version
  (possibly through external scripts, for added flexibility).


Actuators
+++++++++

- The semantics of the :doc:`user/actuators/waypoint` and
  :doc:`user/actuators/destination` actuators has slightly changed: once the
  destination is reached, they no longer actively attempt to stay at this
  position. This allows another motion actuator to 'take over' the control of
  the robot. The previous behaviour is still desirable in certain cases (notably
  for flying robots), and can be re-enabled by setting the property
  ``RemainAtDestination`` to true:
  ``motion.properties(RemainAtDestination=True)``. This option is also added to
  the :doc:`user/actuators/rotorcraft_waypoint` actuator, but it defaults to
  true (hence, no behaviour change compared to MORSE 1.3).
- The :doc:`user/actuators/orientation` actuator has been enhanced to possibly
  work more realistically, by limiting the speed of the rotations. The default
  is still to go directly to the desired orientation.
- The :doc:`user/actuators/keyboard` and :doc:`user/actuators/joystick`
  actuators no longer call the robot's ``apply_speed`` method with values set
  to zero when no input is received. The previous behaviour prevented them from being
  used in combination with another motion actuator (since they would always overwrite
  other motion commands with zeros).
- The :doc:`user/actuators/armature` actuator has two new services
  (``rotate_joints`` and ``translate_joints``) that let the user set the
  rotations/translations of only a subset of the armature's joints by providing
  a custom mapping ``{joint name: value}``.
- The :doc:`user/actuators/rotorcraft_attitude` has been extended to be able
  to control the rotorcraft in yaw rate or in absolute yaw (using the
  ``YawRateControl`` property). So it is now possible 
  to use, for example, ``normal yaw`` or ``north`` using the
  property ``UseAngleAgainstNorth``. Also, you can configure the actuator to
  use a linear or quadratic thrust model using ``LinearThrust``.
- Added a new :doc:`user/actuators/drag` "actuator" which allows for the simulation
  of a drag (air resistance) force opposing the robot's movement. If used, this allows
  more realistic simulations.
- Added a new :doc:`user/actuators/external_force` actuator which can
  apply external force (typically force from the environment such as wind), to
  a robot. It has the same interface as :doc:`user/actuators/force_torque`,
  but applies force in the global frame.
- Added a new :doc:`user/actuators/quadrotor_dynamic_control` actuator which
  supports controlling a quadrotor from the speed of its four engines, using
  a simple dynamic equation.


Sensors
+++++++

- The **longitude**, **latitude** and **altitude** properties are no longer properties of
  :doc:`user/sensors/gps` but must be set at the environment level. Moreover,
  the property **angle_against_north** allows the angle between
  the X-axis and the geographic north to be configured (It must be positive when the Blender
  X-axis is East of true North, negative if is West of true North.).
- Added a new high-level :doc:`user/sensors/attitude` sensor, allowing
  the computation of the system's attitude.
- Added a new :doc:`user/sensors/magnetometer` sensor, allowing the
  computation of the Earth's magnetic field vector.
- Extended the :doc:`user/sensors/imu` sensor, to also return the magnetic field
  vector.
- Fixed the :doc:`user/sensors/collision` sensor: it now detects collisions only
  when it is actually colliding (before, any object in a 1x1x1m box around the
  sensor would return a collision). Also the documentation has been
  improved with the addition of a complete example.
- Added a new :doc:`user/sensors/airspeed` sensor, allowing the computation of
  the speed of a vehicle relative to the air.

Modifiers
+++++++++

- Introduced ECEF, Geodetic, and Geocentric modifiers, allowing the conversion
  of coordinates from Blender world to ECEF-r or Geodetic or Geocentric
  coordinates (and vice-versa). This should improve interoperability with flight
  systems in general.
- Introduced the Feet modifier, to convert imperial units to meter units (and
  vice-versa)

Middlewares
-----------

General
+++++++

- Introduced a binding for the `Mavlink protocol
  <http://qgroundcontrol.org/mavlink/start>`_, easing the interoperability of
  Morse with a lot of free autopilots / architectures.

ROS
+++

- Some ROS 'housekeeping' has been performed in this release, including
  removing the need for ``rospkg`` (easier installation!), removing the ROS interface
  with the non-standard (and unused?) ``JointPositions`` message, and removing
  references to ``roslib.load_manifest()``, a hangover from the rosbuild-era.

HLA
+++

- Now automatically handle the case where attributes published by Morse are not
  owned by it.
- Allow the specification of a ``stop_time`` for the simulation (in simulated seconds)
- Make ``lookahead`` configurable for the Morse federate.

YARP
++++

- Addded an adapter for the :doc:`user/sensors/depth_camera`.

Builder API
-----------

API additions
+++++++++++++

- It is now possible to import environments composed of multiple scenes. The
  user should choose which is the ``main_scene`` when importing the environment.
  In addition, the ``Environment.set_background_scene`` method has been added to configure the
  scene to use as background (`#651 <https://github.com/morse-simulator/morse/issues/651>`_).
- The ``bpymorse.set_speed`` method , used to change the frequency of Morse's
  main loop is now deprecated in favor of the ``Environment.simulator_frequency`` method.
- The ``Environmement.set_time_scale`` method allows the simulation to be accelerated or
  slowed-down (`#388 <https://github.com/morse-simulator/morse/issues/388>`_).
- The new ``Environment.use_vsync`` method allows the vsync parameter to be controlled.


Pymorse
-------

- Robots created in loops are now intelligently handled. They are still usable as
  before, but it is also possible to access them using the list foos (if
  your robot name is foo) (`#358  <https://github.com/morse-simulator/morse/issues/358>`_).
- Streams are now created lazily, fixing control with a large number of robots /
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
