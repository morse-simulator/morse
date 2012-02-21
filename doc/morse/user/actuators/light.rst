Light (On/Off) actuator 
=======================

This actuator is a simple On/Off light. Based on `SPOT 
<http://wiki.blender.org/index.php/Doc:2.6/Manual/Lighting/Lamps/Spot>`_ light.

Since this actuator does not have a Blender file, it can only be added into
a scene using the Builder API. Yo do this by adding these lines into
your script:

.. code-block:: python

  light = morse.builder.actuators.Light()

Files 
-----

-  Blender: no .blend, see: ``morse.builder.actuators.Light``
-  Python: ``$MORSE_ROOT/src/morse/actuators/light.py``

Properties
----------

-  Emit in +X
-  Spot size = 90°
-  Distance = 10m
-  Energy: On = 1.0; Off = 0.0

Local data 
----------

-  **emit**: (boolean) On/Off control


see: `bge types LightObject
<http://www.blender.org/documentation/blender_python_api_2_61_release/bge.types.html#bge.types.KX_LightObject>`_ 
and `Spot Lamps
<http://wiki.blender.org/index.php/Doc:2.6/Manual/Lighting/Lamps/Spot>`_
