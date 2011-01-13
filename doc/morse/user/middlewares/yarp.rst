YARP
====

The YARP middleware provides a simple method to share data across components in
different computers. It is based on the concept of *ports*, which are
registered in a *name server* to differentiate between separate channels of
communication.

Data is shared in the format of bottles, which are nested data structures of
various data types.

.. note:: Port names used in MORSE have the following format:
  ``/ors/robots/[robot_name]/[component_name]/[direction]``. The term [direction]
  is either ``in`` or ``out``, depending on the type of component being actuator
  or sensor, respectively.

Files
-----

- Blender: ``$ORS_ROOT/data/morse/components/middleware/yarp_empty.blend``
- Python: ``$ORS_ROOT/src/morse/modifiers/yarp_mw.py``

Available methods
-----------------

- ``read_message``: Gets information from a port, and stores it in the
  ``modified_data`` array of the associated component. This method is only able
  to handle data of types: integer, float and string.  
- ``post_message``: Formats the contents of ``modified_data`` into a bottle,
  and sends it through the port associated with a component. This method is
  only able to handle data of types: integer, float and string.
- ``post_image_RGBA``: (Requires YARP version 2.5+) Sends an image through a
  specialised port. The image must be stored in ``modified_data`` as a binary
  string with 32bit pixels encoded as RGBA. The actual size of the image is
  read directly from the component instance that called this method.

Known problems
--------------

Some blender files will start the simulation fine the first time after opening the file, but stopping the simulation and starting it again will give this error::

    def open(self, *args): return _yarp.Contactable_open(self, *args)
    NotImplementedError: Wrong number of arguments for overloaded function 'Contactable_open'.

To correct this, it is necessary to open the Blender file, then on a Text
Editor window, select the file ``load_yarp.py``. To the right of the file name,
there is a checkbox named ``Register``. Check this checkbox, save the file, and
open it again. This will ensure that the script ``load_yarp.py`` is read every
time the file is opened, so that yarp is correctly setup.

