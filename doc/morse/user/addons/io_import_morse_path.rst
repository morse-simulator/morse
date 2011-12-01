Import path file into MORSE
===========================

This add-on is accessible from the ``File>Import>Load MORSE path(.pth)`` menu.
When executed, it will open a file browser to select the desired file to load.
It will generate a mesh that represents the points of the path.
Some parameters can be changed to adjust the color and position of the mesh.

Input file
----------

It expects as input files in the ``.pth`` format, which is a plain text file
where each line represents a point in the path. Each line has 3 floating point
numbers, representing the X, Y and yaw values.
Example::

    14.7525 9.55692 0.642678
    14.9612 9.83167 1.17169
    15.0429 10.1759 1.49637
    15.0101 10.6303 1.7825
    14.74 11.3238 2.08953
    14.0898 12.1304 2.40238


Adjustable options
------------------

While the file browser is open, there will be a panel at the bottom left part of
the screen, with the options that can be modified for the path mesh to be generated:

- **Snap to selected object**: If this checkbox is selected, the path will start at
    the position and with the orientation of the currently selected object.
    If this option is not checked, or there is no object selected, then the path will
    be placed at the origin of the Blender world, with the default orientation.

- **Path color**: The color of the path mesh can be selected here.
