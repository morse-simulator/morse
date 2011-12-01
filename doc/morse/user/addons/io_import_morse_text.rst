Import path from text middleware output into MORSE
==================================================

This add-on is accessible from the ``File>Import>Load MORSE path from log(.txt)`` menu.
When executed, it will open a file browser to select the desired file to load.
It will generate a mesh that represents the points of the path.
Some parameters can be changed to adjust the color and density of the mesh.

The resulting path is always placed with respect to the Blender origin.


Input file
----------

It expects as input files generated using the :doc:`text middleware <../middlewares/text>`.
It will extract the position and orientation of the robot stored in such files
for every lecture of the sensor. It does not matter which sensor generated the output,
since all of them produce the necessary data.

Adjustable options
------------------

While the file browser is open, there will be a panel at the bottom left part of
the screen, with the options that can be modified for the path mesh to be generated:

- **Number of frames to skip**: Determines how many of the entries in the log file
    will be discarded and not drawn in the path. This depends on the frequency with which
    each sensor stores the data.
    In most cases, sensors capture data at 60 Hz, which is more than necessary to
    properly draw a path. Increasing the number of frames skipped will make the path mesh
    less cluttered.

- **Path color**: The color of the path mesh can be selected here.
