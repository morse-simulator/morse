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

Code samples
------------

- `Scenario with a PTU from the component unit-test <https://github.com/laas/morse/blob/master/testing/base/ptu_testing.py#L28>`_ :tag:`builder` 
- `Datastream usage, from the component unit-test <https://github.com/laas/morse/blob/master/testing/base/ptu_testing.py#L56>`_ :tag:`pymorse` :tag:`datastream` 
- `Service usage, from the component unit-test <https://github.com/laas/morse/blob/master/testing/base/ptu_testing.py#L121>`_ :tag:`pymorse` :tag:`service` 

Exported data
-------------

-  **pan**: (float) rotation around the Z axis
-  **tilt**: (float) rotation around the Y axis

Configurable parameters
-----------------------

-  **Speed**: (float) rotation speed for the movements of the pan-tilt unit.
   Default value is 1 radian/s.
-  **Tolerance**: (float) the accepted error between the angle of each segment
   and the one specified. If set too low, the actuator will jump back and forth
   between positions close to those indicated. Default value is 0.005 radians.
-  **Manual**: (boolean) select whether to use control from an external
   program or direct control using the **Logic Bricks**.

Services
--------

- **set_pan_tilt**: (asynchronous service) Receives as parameters the expected
  pan and tilt angles (in radians) and finishes when the actuator is effectively
  at this position.

    +------------+---------------+------------------+
    | Parameters | ``pan``       | float            |
    |            +---------------+------------------+
    |            | ``tilt``      | float            |
    +------------+---------------+------------------+

    Parameters: ``(pan, tilt)``


- **get_pan_tilt**: (synchronous service) Returns the current angles of the 
  pan and tilt segments (in radians).

Applicable modifiers 
--------------------

No available modifiers

Files 
-----

-  Blender: ``$MORSE_ROOT/data/actuators/ptu.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/ptu.py``


Related components
------------------

The sensor :doc:`PTU posture <../sensors/ptu_posture>` will read the data from the **get_pan_tilt**
service of this component, providing a constant stream of data.
For them to work, the PTU posture sensor must be set as the parent of the PTU actuator.
It will generate errors if not set in this way.
