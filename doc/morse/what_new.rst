What's new in MORSE 0.4?
========================

MORSE 0.4 has been code-named the ``multi-node release``.

General changes
---------------
- Full support for Blender 2.59 and Python 3.2. See the :doc:`updated installation instructions <user/installation>`.

Architectural changes
---------------------
- Added infrastructure for :doc:`multi-node <multinode>` functionality
- Changed directory structure and file names for ease of use of the Builder API.
- Use the Builder API to create robots from their description in Python script
- Use the Python logging interface

Middlewares
-----------
- Added support for multiple middleware bindings per component
- Added support for services through Pocolibs middleware
- ROS support for the robot's cameras
- Added support for HLA middleware
- Added support for MOOS middleware

New components
--------------
- Added a Hummer robot that implements the Blender Vehicle Wrapper
- Added a ``steer_force`` actuator to control the Hummer robot
- Added an ``armature_actuator`` to control the bone structure in the LWR and PR2 robot arms

User interface
--------------
- Add a help display with the keyboard shortcuts available during simulation. Activated by pressing the ``H`` key
