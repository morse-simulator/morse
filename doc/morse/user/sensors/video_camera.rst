Video camera sensor
===================

This sensor emulates a single video camera. It generates a series of RGBA images.
Images are encoded as binary char arrays, with 4 bytes per pixel.

The cameras make use of Blender's **bge.texture** module, which requires
a graphic card capable of GLSL shading.
Also, the 3D view window in Blender must be set to draw **Textured** objects.

.. note::
    The streaming of data from this sensor can be toggled off and on by
    pressing the SPACE key during the simulation. This will affect all the
    video cameras on the scene.

    Toggling off the cameras can help make the simulation run faster,
    specially when there are several cameras. However, the lack of
    data on the stream may cause problems to some middlewares.


.. image:: ../../../media/sensors/video_camera.png 
  :align: center
  :width: 600

Files
-----

- Blender: ``$MORSE_ROOT/data/sensors/video_camera.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/video_camera.py``


Local data
----------

- **image**: The data captured by the camera, stored as a Python Buffer class
  object. The data is of size ``(cam_width X cam_height * 4)`` bytes. The image
  is stored as RGBA.
- **intrinsic_matrix**: The intrinsic calibration matrix, stored as a 3x3
  row major Matrix

Configurable parameters
-----------------------

The Empty object corresponding to this sensor has the following parameters:

- **capturing**: (Boolean) flag that determines whether the camera should
  generate an image. It can be toggled on or off by pressing the :kbd:`Space`.
  The default value should be `True`.
- **cam_width**: (int) generated image width in pixels. The default value is
  `256`.
- **cam_height**: (int) generated image height in pixels. The default value is
  `256`.
- **cam_focal**: (double) camera focal length as defined in Blender (note: in
  Blender this parameter unit is "millimeters". This is actually misleading, as
  there is no dimension associated to Blender units.) The default value is
  `25.0`.
- **cam_near**: (double) Adjust the `near_clipping` parameter of the camera
  (Anything closer to the camera than `near_clipping` is not displayed). The
  default value is `0.1` (meter).
- **Vertical_Flip**: (Boolean) flag that determines whether the image should be 
  flipped. Default value is `False`.

Camera calibration matrix
-------------------------

The camera configuration parameters implicitly define a geometric camera in
blender units. Knowing that the **cam_focal** attribute is a value that
represents the distance in Blender unit at which the largest image dimension is
32.0 Blender units, the camera intrinsic calibration matrix is defined as

  +--------------+-------------+---------+-------+
  | **alpha_u**  |      0      | **u_0** | 0     |
  +--------------+-------------+---------+-------+
  |       0      | **alpha_v** | **v_0** | 0     |
  +--------------+-------------+---------+-------+
  |       0      |      0      |    1    |   0   |
  +--------------+-------------+---------+-------+

where:

- **alpha_u** == **alpha_v** = **cam_width** . **cam_focal** / 32 (we suppose
  here that **cam_width** > **cam_height**. If not, then use **cam_height** in
  the formula)
- **u_0** = **cam_height** / 2
- **v_0** = **cam_width** / 2

Services
--------

- **capture**: (asynchronous service) the method expects an integer **n** in
  input and answer back when the simulated camera has token **n** shots.

Applicable modifiers
--------------------

No camera modifiers available at the moment

Related components
------------------

Two or more cameras can be mounted on a :doc:`stereo unit <stereo_unit>` so that the images they
generate can be processed together.
Cameras or stereo units can also be parented to a :doc:`pan-tilt unit <../actuators/ptu>` so they
can be oriented during the simulation.
