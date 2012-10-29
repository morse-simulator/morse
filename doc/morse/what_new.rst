MORSE 0.6
=========

General
-------

- Lots of improvents on ROS compatibility. Many new tutorials with detailed
  explanations
- Compatibility with Blender 2.63a and 2.64 
- MORSE is now compatible with Windows 32 and 64 bit. Thanks to Markus Sander
  for providing the patches and testing
- Improvements in human-robot interaction scenarios
- The 'morse' executable has slightly different options now. run 'morse -h' for
  details.
- Added support for 'no color' and 'reverse colors' log output.
- Added support for specifying the geometry of the simulator window.
- Improvements to the multi-node architecture using HLA. Including new
  tutorials and documentation
- Add methods in builder to configure UTM coordinates and temperature in the
  scene. Previously in the Scene_Script_Holder
- Improve unit-tests coverage

User interface
--------------

- Possibility to configure and display the view from a simulated camera inside
  the Blender screen
- Reset the position of the global camera (CameraFP) by pressing F8

Components
----------

- New more physically realistic robots: Segway RMP 400 and Pioneer 3-DX. Thanks
  to David Hodo and Pierrick Koch for their work on the physics simulation
- New differential drive actuator associated to the previously mentioned
  robots, called 'v_omega_diff_drive'. It converts a given v, omega into left
  and right wheel speeds
- Waypoint actuator can be configured to give target destination also in the Z
  axis. Useful for helicopters and submarines
- Human avatar can now be correctly placed in the scene using the Builder API
  scripts
- New textured model for the Yamaha R-Max helicopter
- Corrections to the bounding boxes of buildings in outdoor scenarios. Also
  added textures to the buildings
- Several models for quadrotors, including more or less realistic controls
  (using waypoints, stabilized fly model or directly in force)
- Major rewriting of the IMU sensor and odometry sensor, which now returns more 
  precise datas. While here, add some modifiers to allow more realistic
  behaviour of such sensors.

Middlewares
-----------

- Corrections to YARP middleware, allowing it to export data stored as Python
  lists

Documentation
-------------

- Make table of contents of the components with images

Previous releases
-----------------

- :doc:`0.4.x release notes <releasenotes/0.4>`
- :doc:`0.5.x release notes <releasenotes/0.5>`
