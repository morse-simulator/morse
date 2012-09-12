Laser range scanners
====================

This is a whole collection of sensors that emulate laser range scanners,
including a variety of SICK and Hokuyo sensors.

This sensor works by generating a series of rays in predefined directions, and
then computing whether any active object is found within a certain distance
from the origin of the sensor.

The resolution and detection range can be completely configured using the MORSE
Builder API.  This will generate a flat mesh with a semi-circular shape, where
its vertices represent the directions in which the rays of the sensor are cast.
It is also possible to create a sensor with multiple scan layers, such as the
SICK LD-MRS. This is configured using the parameters specified below.

.. note:: Objects in the scene with the **No collision** setting in their Game
  properties will not be detected by this sensor


+----------------------------------------------------+----------------------------------------------------+
| .. figure:: ../../../media/sensors/sick.png        | .. figure:: ../../../media/sensors/sick-ld-mrs.png |
|    :width: 400                                     |    :width: 400                                     |
|                                                    |                                                    |
|    SICK LMS500                                     |    SICK LD-MRS                                     |
+----------------------------------------------------+----------------------------------------------------+
|.. figure:: ../../../media/sensors/hokuyo.png       |                                                    |
|   :width: 400                                      |                                                    |
|                                                    |                                                    |
|   Hokuyo                                           |                                                    |
+----------------------------------------------------+----------------------------------------------------+

Files
-----

There are a few different Blender files available for this type of sensor, with
meshes representing various models of laser range scanners. They all use the
same Python script for their behaviour.

- Blender: ``$MORSE_ROOT/data/sensors/sick.blend``,
  ``$MORSE_ROOT/data/sensors/sick-ld-mrs.blend``,
  ``$MORSE_ROOT/data/sensors/hokuyo.blend``
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
  is displayed during the execution of the simulation or not (Default: *False*).
  If the robot is also equipped with a camera, it is better to set this
  variable to *False*, otherwise the scanned area will also appear on the
  captured images.
- **laser_range**: (Float) The distance in meters from the center of the sensor
  to which it is capable of detecting other objects.
- **resolution**: (Float) The angle between each laser in the sensor. Expressed
  in degrees in decimal format. (*i. e.*), half a degree is expressed as 0.5.
  Used when creating the arc object.
- **scan_window**: (Float) The full angle covered by the sensor. Expressed in
  degrees in decimal format. Used when creating the arc object.

The following parameters are optional. Mainly used to create multiple scanning
layers, as is the case for the SICK LD-MRS sensor:

- **layers**: (Integer) Number of scanning planes used by the sensor.
- **layer_separation**: (Float) The angular separation between the planes. Must
  be given in degrees.
- **layer_offset**: (Float) The horizontal distance between the scan points in
  consecutive scanning layers. Must be given in degrees.

Configuration of the scanning parameters
++++++++++++++++++++++++++++++++++++++++

The number and direction of the rays emitted by the sensor is determined by the
vertices of a semi-circle mesh parented to the sensor. The sensor will cast
rays from the center of the sensor in the direction of each of the vertices in
the semi-circle.

By default, the SICK has an arc that spans 180 degrees, with a resolution of 1
degree.  This can be changed using a special method in the Builder API, which
can create any arc of the desired geometry. The shape of the arc is determined
from the parameters of **resolution** and **scan_window** specified as game
properties of the Sick sensor. The name of the method is ``create_sick_arc``,
and must be called after setting the desired value for the previously mentioned
properties.

An example of how to change the arc object using the Builder API is show below:

.. code-block:: python

    from morse.builder import *

    # Append a sick laser
    sick = Sensor('sick')
    sick.properties(resolution = 5)
    sick.properties(scan_window = 90)
    sick.properties(laser_range = 5.0)
    sick.create_sick_arc()


An example for creating a properly configured SICK LD-MRS is given below:

.. code-block:: python

    from morse.builder import *

    sick = Sensor('sick-ld-mrs')
    sick.properties(Visible_arc = True)
    sick.properties(resolution = 1.0)
    sick.properties(scan_window = 100)
    sick.properties(laser_range = 50.0)
    sick.properties(layers = 4)
    sick.properties(layer_separation = 0.8)
    sick.properties(layer_offset = 0.25)
    sick.create_sick_arc()

As with any other component, it is possible to adjust the refresh frequency of
the sensor, after it has been defined in the builder script. For example, to
set the frequency to 1 Hz:

.. code-block:: python

    sick.frequency(1.0)
