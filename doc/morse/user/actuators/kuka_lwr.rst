KUKA LWR arm actuator
=====================

This actuator reads a list of angles for the segments of the LWR arm
and applies them as local rotations.
It is a subclass of the :doc:`armature_actuator <armature_actuator>`.
Angles are expected in radians.

To install additional components at the tip of the arm using the
MORSE Builder API, it is necessary to make the additional component as a
child of the arm, and to place the component in the correct position with
respect to the kuka arm.
Example::

    kuka_arm = Actuator('kuka_lwr')
    kuka_arm.translate(x=0.1850, y=0.2000, z=0.9070)
    kuka_arm.rotate(x=1.5708, y=1.5708)
    Jido.append(kuka_arm)

    gripper = Actuator('gripper')
    gripper.translate(z=1.2800)
    kuka_arm.append(gripper)

When the simulation is started any objects that are children of the KUKA arm
will automatically be changed to be children of the last segment of the arm.


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

Use of the KUKA LWR
-------------------

A sample python script of how to access the KUKA LWR via sockets can be found at:
``$MORSE_ROOT/examples/morse/scenarii/armature_samples/armature_services_tests.py``.
