====== Straight line movement ======

This actuator reads the coordinates of a destination point, and moves the robot in a straight line towards the given point, without turning.
It provides a very simplistic movement, and can be used for testing or for robots with holonomic movement.
The speeds provided are internally adjusted to the Blender time measure.

===== Files =====
  * Blender: ''$ORS_ROOT/data/morse/components/controllers/morse_destination_control.blend''
  * Python: ''$ORS_ROOT/src/morse/actuators/destination.py''

===== Local data =====
  * **x**: Destination X coordinate
  * **Y**: Destination Y coordinate
  * **Z**: Destination Z coordinate

__Note__: Coordinates are given with respect to the origin of Blender's coordinate axis.

===== Applicable modifiers =====

  * UTM modifier: Will add an offset to the Blender coordinates according to the parameters set on the scene.

  * NED: Changes the coordinate reference to use North (X), East (Y), Down (Z)
