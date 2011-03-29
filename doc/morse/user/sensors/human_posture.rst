Human posture sensor
====================

This sensor collects the positions of the bones in the human armature
for the file ``$MORSE_ROOT/data/morse/components/human/default_human.blend``.

It stores the position and orientation of the general armature object, as
well as the local rotation of each individual bone. The rotation angles are
given in radians.

This sensor will only work for the ``default_human.blend`` model, as it uses
a specific naming convention for each of the bones.

You can also check to general documentation of the :doc:`human component <../others/human>`.

Files
-----

- Blender: ``$MORSE_ROOT/data/morse/components/human/default_human.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/human_posture.py``

Local data
----------

.. image:: ../../../media/human_joints.png 
  :align: center
  :width: 600


- **x**: (float) global X position of the armature in the scene
- **y**: (float) global Y position of the armature in the scene
- **z**: (float) global Z position of the armature in the scene
- **yaw**: (float) rotation angle with respect to the Z axis
- **pitch**: (float) rotation angle with respect to the Y axis
- **roll**: (float) rotation angle with respect to the X axis

- **dof_12**: (float) rotations around the X axis for the torso
- **dof_13**: (float) rotations around the Y axis for the torso
- **dof_14**: (float) rotations around the Z axis for the torso

- **dof_15**: (float) rotations around the X axis for the head
- **dof_16**: (float) rotations around the Y axis for the head
- **dof_17**: (float) rotations around the Z axis for the head

- **dof_18**: (float) rotations around the X axis for the right shoulder
- **dof_19**: (float) rotations around the Y axis for the right shoulder
- **dof_20**: (float) rotations around the Z axis for the right shoulder

- **dof_21**: (float) rotations around the Z axis for the right elbow

- **dof_22**: (float) rotations around the X axis for the right hand
- **dof_23**: (float) rotations around the Y axis for the right hand
- **dof_24**: (float) rotations around the Z axis for the right hand

- **dof_25**: (float) rotations around the X axis for the left shoulder
- **dof_26**: (float) rotations around the Y axis for the left shoulder
- **dof_27**: (float) rotations around the Z axis for the left shoulder

- **dof_28**: (float) rotations around the Z axis for the left elbow

- **dof_29**: (float) rotations around the X axis for the left hand
- **dof_30**: (float) rotations around the Y axis for the left hand
- **dof_31**: (float) rotations around the Z axis for the left hand

- **dof_32**: (float) rotations around the X axis for the right hip
- **dof_33**: (float) rotations around the Y axis for the right hip
- **dof_34**: (float) rotations around the Z axis for the right hip

- **dof_35**: (float) rotations around the Z axis for the right knee

- **dof_36**: (float) rotations around the X axis for the right foot
- **dof_37**: (float) rotations around the Y axis for the right foot
- **dof_38**: (float) rotations around the Z axis for the right foot

- **dof_39**: (float) rotations around the X axis for the left hip
- **dof_40**: (float) rotations around the Y axis for the left hip
- **dof_41**: (float) rotations around the Z axis for the left hip

- **dof_42**: (float) rotations around the Z axis for the left knee

- **dof_43**: (float) rotations around the X axis for the left foot
- **dof_44**: (float) rotations around the Y axis for the left foot
- **dof_45**: (float) rotations around the Z axis for the left foot

Applicable modifiers
--------------------

No applicable modifiers at the moment.

Related components
------------------

This sensor will only be useful for the armature of the human in the
``default_human.blend`` file. The human in this file is also considered as a
robot, with the sensor integrated into the armature. Check the general 
documentation of the :doc:`human component <../others/human>`.

It is possible to reuse this sensor in other human models, as long as they
use the same armature.
