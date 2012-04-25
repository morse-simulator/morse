SICK laser range scanner
========================

This sensor emulates a laser range scanner, by generating a series of rays in
predefined directions, and then computing whether they find any active object
within a certain distance of the sensor's origin.

The rays cast by the sensor are determined by the geometry of a flat mesh with
a semi-circular shape. There must be a mesh whose name starts with ``Arc_`` and
is a child of the Sick sensor. The sensor will cast rays in the direction of
the vertices of this mesh. Because of this, it is imperative that the mesh is
correctly configured.

.. note:: Objects in the scene with the **No collision** setting in their Game
  properties will not be detected by this sensor


.. image:: ../../../media/sensors/sick.png 
  :align: center
  :width: 600

Files
-----

- Blender: ``$MORSE_ROOT/data/sensors/sick.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/sick.py``

Local data
----------

- **point_list**: (list array) Array that stores the positions of the points
  found by the laser. The points are given with respect to the location of the
  sensor, and stored as lists of three elements. The number of points depends
  on the geometry of the arc parented to the sensor (see below).
- **range_list**: (float array) Array that stores the distance to the first
  obstacle detected by each ray. The order indexing of this array is the same
  as for **point_list**, so that the element in the same index of both lists
  will correspond to the measures for the same ray.

Configurable Parameters
-----------------------

The Empty object corresponding to this sensor has the following parameters
in the **Logic Editor** panel:

- **Visible_arc**: (Boolean) A toggle that determines whether the scanned area
  is displayed during the execution of the simulation or not (Default: True).
  If the robot is also equipped with a camera, it is better to set this
  variable to False, otherwise the scanned area will also appear on the
  captured images.
- **laser_range**: (Float) The distance in meters from the center of the sensor
  to which it is capable of detecting other objects.
- **resolution**: (Float) The angle between each laser in the sensor. Expressed
  in degrees in decimal format. (*i. e.*), half a degree is expressed as 0.5.
  Used when creating the arc object.
- **scan_window**: (Float) The full angle covered by the sensor. Expressed in
  degrees in decimal format. Used when creating the arc object.

Number and angle of rays
++++++++++++++++++++++++

The number and direction of the rays emitted by the sensor is determined by the
use of a semi-circle object parented to the sensor. The sensor will cast rays
from the center of the sensor in the direction of each of the vertices in the
semi-circle.

By default, the Sick sensor will be created with an arc that spans 180 degrees,
with a resolution of 1 degree.
This can be changed using a special method in the Builder API, which can create
any arc of the desired geometry. The shape of the arc is determined from the
parameters of **resolution** and **scan_window** specified as game properties
of the Sick sensor. The name of the method is ``create_sick_arc``, and must be
called after setting the desired value for the previously mentioned properties.

The new arc object will have the following characteristics (all of them are
correctly configured by the ``create_sick_arc`` method):

- Name: Its name must begin with 'Arc\_', for the SICK Module to recognize it.
  The currently used method is to name the arcs according to the number of
  degrees it covers, for example: Arc_180, Arc_16, Arc_360
- Normals: For the semi-circle to be visible in the Game Engine, the normals of
  the faces must be facing up. Otherwise the object will not be displayed 
- Physics: Make sure that on the **Physics Properties** panel this object is
  set to **No collision**, otherwise it will push objects around

An example of how to change the arc object using the Builder API is show below:

.. code-block:: python

    from morse.builder.morsebuilder import *

    # Append a sick laser
    sick = Sensor('sick')
    sick.properties(resolution = 5)
    sick.properties(scan_window = 90)
    sick.properties(laser_range = 5.0)
    sick.create_sick_arc()
