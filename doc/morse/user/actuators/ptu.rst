Pan-Tilt Unit actuator
======================

This actuator reads the rotation values for pan and tilt, and applies
them to the pan-tilt unit that must be set as children of this actuator.
Angles are expected in radians.

Unlike most other actuators, the Pan-Tilt unit is composed not only of an
Empty object, but it also includes two meshes. These are the **PanBase** and
the **TiltBase** that must also be imported when using this actuator.
These meshes will rotate to produce the effect of a real Pan-Tilt unit.

.. note:: When mounting a camera or stereo unit on top of the Pan-Tilt unit,
    make sure to parent the camera to the **PTU** object.

This component can be configured to be operated manually as well as through data
from a middleware. When using manual mode, the pan and tilt segments can be rotated
using the following keys:

-  :kbd:`Page Up` tilt up
-  :kbd:`Page Down` tilt down
-  :kbd:`Home` pan left
-  :kbd:`Insert` pan right


Files 
-----

-  Blender: ``$MORSE_ROOT/data/morse/actuators/ptu.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/ptu.py``

Local data 
----------

-  **pan**: (float) rotation around the Z axis
-  **tilt**: (float) rotation around the Y axis

Configurable parameters
-----------------------

-  **Speed**: (float) rotation speed for the movements of the pan-tilt unit
-  **Manual**: (boolean) select whether to use control from an external
   program or direct control using the **Logic Bricks**

Services
--------

- **set_pan_tilt**: (asynchronous service) This method take in parameters the
  wished pan and tilt (in radians) and finish when the actuator is effectively
  at this position.

- **get_pan_tilt**: (synchronous service) Returns the current angles of the 
  pan and tilt segments (in radians).

Applicable modifiers 
--------------------

No available modifiers
