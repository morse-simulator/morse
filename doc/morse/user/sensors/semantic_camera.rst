Semantic camera sensor
======================

This sensor emulates a hight level camera that outputs the names of the objects
that are located within the field of view of the camera.

The sensor determines first which objects are to be tracked (objects marked with
a **Logic Property** called ``Object``, cf documentation on :doc:`passive
objects <../others/passive_objects>` for more on that). If the ``Label`` property
is defined, it is used as exported name. Else the Blender object name is used.

Then a test is made to identify which of these objects are inside of the view
frustum of the camera. Finally, a single visibility test is performed by casting
a ray from the center of the camera to the center of the object. If anything
other than the test object is found first by the ray, the object is considered
to be occluded by something else, even if it is only the center that is being
blocked.

The cameras make use of Blender's **bge.texture** module, which requires a
graphic card capable of GLSL shading. Also, the 3D view window in Blender must be
set to draw **Textured** objects.

Files
-----

- Blender: ``$MORSE_ROOT/data/sensors/semantic_camera.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/semantic_camera.py``


Local data
----------

- **visible_objects**: (List) An array containing the description of objects
  visible by the camera. Objects are described as a dictionary containing:

	- **name**: (String) the name of the object
	- **type**: (String) the type of the object
	- **position**: (Vector3) the position of the object (x, y, z)
	- **orientation**: (Quaternion) the orientation of the object (w, x, y, z)

Configurable parameters
-----------------------

The Empty object corresponding to this sensor has the following parameters:

- **capturing**: (Boolean) flag that determines whether the camera should
  generate an image. It can be toggled on or off by pressing the :kbd:`Space`
- **cam_width**: (double) generated image width in pixels
- **cam_height**: (double) generated image height in pixels
- **cam_focal**: (double) camera focal length as defined in Blender.
  Not really important for this kind of camera.

Applicable modifiers
--------------------

No camera modifiers available at the moment

Related components
------------------

The movement of the semantic camera is implemented by making it the child of a
:doc:`Pan-Tilt unit <../actuators/ptu>` actuator.
