What's new in MORSE 1.3?
========================

General
-------

Components
----------

Robots
++++++

Actuators
+++++++++

- Introduce the new actuator :doc:`user/actuators/arucomarker`, allowing to
  simulate the ArUco augmented-reality marker.

Sensors
+++++++

- The timestamp field is now in seconds instead of milliseconds (`#498 <https://github.com/morse-simulator/morse/issues/498>`_)

- :doc:`user/sensors/semantic_camera` gains two properties (`#396 <https://github.com/morse-simulator/morse/issues/396>`_):
    - `tag` allows to restrict the kind of object you want to detect
    - `relative` returns the position information of the various objects from
      the camera sensor frame (and not the global frame).

- :doc:`user/sensors/laserscanner` gain the possibility to return also a
  remission value at the `rssi` level.

- Introduce the new sensor :doc:`user/sensors/radar_altimeter`, allowing to
  retrieve the distance to the ground.

Middlewares
-----------

General
+++++++

- Each datastream manager now get an action handler, allowing them to run some
  specific middleware behaviour once by simulation turn.

Socket
++++++

- Socket middleware now accepts the keyword 'port' to specify on which port
  you want the socket binds itself.


Builder API
-----------

API addition
++++++++++++

- Add a method ``Environment.configure_stream_manager`` allowing to pass
  option/information to each datastream manager.

What's new in MORSE 1.2?
========================

General
-------

- Time management in Morse has been clarified (#388). See
  :doc:`time management documentation <../dev/time_event>` for more details about it.
- Implement the notion of zone, i.e. a 3d space which several properties which
  can trigger various behaviours in the simulation. See :ref:`here
  <define_new_zone>` to see how to define zone in your scenario.
- Add new services for the CamaraFP settings.
- F7 moves CamaraFP above robots

Components
----------

- Each component now has two new services:
    - ``get_properties`` returns the list of the properties of the component
    - ``get_configurations`` returns the value of the different properties of
      the component.

Robots
++++++

- Most robots are now using more realistic physical behaviour.

Actuators
+++++++++

- :doc:`Armatures <user/actuators/armature>` have received some love, with
  support for placing and controlling inverse kinematics targets to easily
  control the full skeleton with inverse kinematics.
- The default ``ControlType`` of several actuators
  (:doc:`user/actuators/v_omega`, :doc:`user/actuators/waypoint`,
  :doc:`user/actuators/xy_omega`, :doc:`user/actuators/keyboard`,
  :doc:`user/actuators/joystick`) has been switched from "Position" to
  "Velocity". It basically means it relies more on the underlaying physic
  engine, providing a more realistic behaviour, but it may be less repeatable.
  The previous behaviour can be restored by setting explicitly the
  ``ControlType`` parameter of the actuator (`#117
  <https://github.com/morse-simulator/morse/issues/117>`_).

Sensors
+++++++

- Each sensor has now an additional field ``timestamp`` exporting when the
  data has been computed, in simulated time.
- the :doc:`user/sensors/gps` has been vastly improved. In addition to (x, y,
  z) position, it can also returns geodesic coordinates and velocity in the
  ``raw`` level of details. Moreover, it exposes also time and heading in the
  ``extended`` level.
- :doc:`Batteries <user/sensors/battery>` are now rechargeable in
  ``ChargingZone``.

Middlewares
-----------

Pocolibs
++++++++

- ``pocolibs`` is now able to export :doc:`velodyne <user/sensors/depth_camera>`
  sensor.

Socket
++++++

- Add a new ``DepthCamera`` publisher.
- ``VideoCamera`` now publish base64 encoded RGBA image.

Builder API
-----------

API changes
+++++++++++

- ``place_camera`` and ``aim_camera`` has been deprecated in favor of
  ``set_camera_location`` and ``set_camera_rotation``. 
- ``Velodyne`` became ``VelodyneRayCast`` and ``VelodyneZB`` became ``Velodyne``
  ``VelodyneZB`` still works for compatibility.

API addition
++++++++++++

- Add a method ``Environment.set_physics_step_sub`` allowing to control the
  number of substep used by the physics engine. A bigger number will make the
  simulation slower, but more realistic. The default value in Morse is 2.

Pymorse
-------

API addition
++++++++++++

- Add two methods ``sleep`` and ``time`` to handle time-related request. These
  methods are equivalent to the one provided by the ``Time`` module, but
  considers properly the simulated time. It is recommended to use these
  methods over ``Time`` one.


Previous releases
-----------------

.. toctree::
    :maxdepth: 1	

    releasenotes/1.1
    releasenotes/1.0
    releasenotes/0.6
    releasenotes/0.5
    releasenotes/0.4
