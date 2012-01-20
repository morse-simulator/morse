What's new in MORSE 0.5.x?
==========================

General
-------

- MORSE 0.5 requires Blender >= 2.59 and < 2.62 (because of some changes in the
  matrices handling - support for Blender >= 2.62 is expected for next release)
  - Lots of cleaning (middleware empties have been removed)
- The command line ``morse run {scene.blend|scene.py}`` now works as expected (ie,
  starts the simulation. Optional arguments can be added and are passed to the script
- Unit-testing support for MORSE (cf doc: dev/testing). Added a target to the build file
  ('make test').

GUI
---

- First version of a graphical user interface to add components to a scene
- Plugin for loading DTM/IGN data has been ported to Blender 2.5/Python 3.2

Middlewares
-----------

- Support of ROS services. Partial support for ROS actions (cf commit 02fda)
- The long-standing issue with the socket server (bug #162) has been solved. It
  is now possible to listen to a socket stream without prior initialization.

Builder API
-----------

- New export script (available as Blender add-on) to export a MORSE Blender
  scene to the MORSE Builder format.
- Added support for multi-node configuration in the builder API
- Added support for static, passive objects

- Many examples and tutorials have been converted to the Builder API.

Components
----------

- Static objects have a redefined set of options to make them active or not,
  graspable or not, etc. See doc: user/others/passive_objects)
- New infrared sensor
- Camera images can now be vertically flipped via the ``vertical_flip`` property.

Multi-node
----------

- New abstract API for multi-node implementation. The current socket-based and HLA
  implementation now use it.

HRI
---

Much work has been done in this domain:

- New human avatar with a much improved behaviour/animation. It is controlable
  from mouse + keyboard or Kinect
- The avatar features an 'manipulation mode' where objects can be picked and
  dropped, and special objects like drawers and cupboards can be opened.
- The human avatar can be easily be added via the Builder API (instantiate the
  'Human' class)

Previous releases
-----------------

- :doc:`0.4.x release notes <releasenotes/0.4>`
