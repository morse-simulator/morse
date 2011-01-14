SICK laser range scanner
========================

This sensor emulates a laser range scanner, by generating a series of rays in
predefined directions, and then computing whether they find any object within a
certain distance of the sensor's origin.

.. note:: Objects in the scene with the **No collision** setting in their Game
  properties will not be detected by this sensor

Files
-----

- Blender: ``$MORSE_ROOT/data/morse/components/sensors/morse_sick.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/sick.py``

Local data
----------

- **point_list**: (float array) Array that stores the positions of the points
  found by the laser. The points are given with respect to the location of the
  sensor, and stored as lists of three elements. The number of points depends
  on the geometry of the arc parented to the sensor (see below).

Configurable Parameters
-----------------------

The Empty object corresponding to this sensor has the following parameters:

- **Visible_arc**: (Boolean) A toggle that determines whether the scanned area
  is displayed during the execution of the simulation or not. If the robot is
  also producing camera images, it is better to set this variable to False,
  otherwise the scanned area will also appear on the captured images.
- **laser_range**: (Float) The distance in meters from the center of the sensor
  to which it is capable of detecting other objects.
- **resolution**: (Float) The angle between each laser in the sensor. Expressed
  in degrees in decimal format. (i. e.), half a degree is expressed as 0.5
- **scan_window**: (Float) The full angle covered by the sensor. Expressed in
  degrees in decimal format.

Number and angle of rays
++++++++++++++++++++++++

The number and direction of the rays emitted by the sensor is determined by the
use of a semicircle object parented to the sensor. The sensor will cast rays
from the center of the sensor in the direction of each of the vertices in the
semicircle.
There is a Python script that can be used to create a new arc with the
parameters of **resolution** and **scan_window** specified for the sensor.
The script is called ``create_arc.py`` and is included in the Blender file for
the sensor. To run it, specify the parameters desired, then in a Text Window
select the script, place the mouse cursor inside the Text Window and press
:kbd:`Alt-p`. This will generate a new arc with the correct geometry, and with
an adequate name. At this point it is only necessary to assign a material to
the arc.

The new arc object will have the following characteristics:

- Name: Its name must begin with 'Arc\_', for the SICK Module to recognize it.
  The currently used method is to name the arcs according to the number of
  degrees it covers, for example: Arc_180, Arc_16, Arc_360
- Normals: For the circle to be visible in the GE, the normals of the faces
  must be facing up. Otherwise the object will not be displayed 
- Physics: Make  sure that on the **Logic Panel** this object is set to **No collision**,
  otherwise it will push objects around

When adding the sick sensor to a robot, make sure to link in the corresponding
arc object.
