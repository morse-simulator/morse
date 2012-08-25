Depth camera sensor
===================

This sensor can be used to simulate laser range sensors. It uses the z-buffer of
a Blender camera to generate a depth image and then process it to produce a list
of 3D points, given with respect to the position of the camera.

The cameras make use of Blender's **bge.texture** module, which requires
a graphic card capable of GLSL shading.
Also, the 3D view window in Blender must be set to draw **Textured** objects.

This sensor requires the generation of z-buffer images instead of the usual
RGBA. For this reason, exporting the images will result in strange (and quite
Psychodelic) colours.

The settings for the ``near`` and ``far`` planes of the camera are important
for this sensor. They will determine the resolution of the depth data
recovered. For best results, it is recommended to keep the ``near`` plane as
far as possible, and the ``far`` plane as close as possible. Reducing the range
between both will result in higher resolution, specially towards the ``far``
plane. For more information:
http://www.codermind.com/articles/Depth-buffer-tutorial.html

.. note::
    The streaming of data from this sensor can be toggled off and on by
    pressing the SPACE key during the simulation. This will affect all the
    video cameras on the scene.

    Toggling off the cameras can help make the simulation run faster,
    specially when there are several cameras. However, the lack of
    data on the stream may cause problems to some middlewares.


.. image:: ../../../media/sensors/depth_camera.png 
  :align: center
  :width: 600

Files
-----

- Blender: ``$MORSE_ROOT/data/sensors/depth_camera.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/depth_camera.py``


Local data
----------

- **3D_points**: A list of lists. Each element of the list has the format
  ``[x, y, z]``, with the coordinates given with respect to the origin of
  the camera.
- **intrinsic_matrix**: The intrinsic calibration matrix, stored as a 3x3
  row major Matrix

Configurable parameters
-----------------------

The Empty object corresponding to this sensor has the following parameters:

- **capturing**: (Boolean) flag that determines whether the camera should
  generate an image. It can be toggled on or off by pressing the :kbd:`Space`.
  The default value should be `True`.
- **cam_width**: (int) generated image width in pixels. The default value is
  `128`.
- **cam_height**: (int) generated image height in pixels. The default value is
  `128`.
- **cam_focal**: (double) camera focal length as defined in Blender (note: in
  Blender this parameter unit is "millimeters". This is actually misleading, as
  there is no dimension associated to Blender units.) The default value is
  `35.0`.
- **cam_near**: (double) Adjust the `near_clipping` parameter of the camera
  (Anything closer to the camera than `near_clipping` is not displayed). The
  default value is `1.0` (meter).
- **cam_far**: (double) Adjust the `far_clipping` parameter of the camera
  (Anything further from the camera than `far_clipping` is not displayed). The
  default value is `20.0` (meter).
- **Vertical_Flip**: (Boolean) flag that determines whether the image should be 
  flipped. Default value is `False`.
- **Depth**: (Boolean) flag that determines whether the sensor generates
  z-buffer images or plain images. It should be set to `True` for this sensor
  to work properly. In this case, the images generated will not be really
  ``Visible``, since the data will be stored as floats, instead
  of the usual RGBA strings. Default value is `True`.

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

Cameras can be parented to a :doc:`pan-tilt unit <../actuators/ptu>` so they
can be oriented during the simulation.
