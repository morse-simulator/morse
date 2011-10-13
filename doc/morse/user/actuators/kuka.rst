KUKA LWR arm actuator
=====================

This actuator reads a list of angles for the segments of the LWR arm
and applies them as local rotations.
It is a subclass of the :doc:`armature_actuator <armature_actuator>`.
Angles are expected in radians.

Files 
-----

-  Blender: ``$MORSE_ROOT/data/morse/robots/kuka_lwr.blend``

   Unlike other actuators, this one also includes the mesh of the arm
   (composed of 8 segments) and an armature that controls its movement.

-  Python: ``$MORSE_ROOT/src/morse/actuators/kuka_lwr.py``

Local data 
----------

.. image:: ../../../media/kuka_joints.png 
  :align: center
  :width: 300

There are 7 floating point values, named after the bones in the armature:

-  **Bone**: (float) rotation for the first segment. Around Z axis.
-  **Bone.001**: (float) rotation for the second segment. Around Y axis.
-  **Bone.002**: (float) rotation for the third segment. Around Z axis.
-  **Bone.003**: (float) rotation for the fourth segment. Around Y axis.
-  **Bone.004**: (float) rotation for the fifth segment. Around Z axis.
-  **Bone.005**: (float) rotation for the sixth segment. Around Y axis.
-  **Bone.006**: (float) rotation for the seventh segment. Around Z axis.

These names are generated dynamically, so that if there are more than one arm
in the scene, there will not be any conflicts.

Configurable parameters
-----------------------

No configurable parameters

Applicable modifiers 
--------------------

No available modifiers


Available services
------------------

See the documentation for the :doc:`armature_actuator <armature_actuator>`.
