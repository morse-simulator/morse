How to control armatures in the blender game engine (2.5*) 
==========================================================

Benoit Bolsee added support for controlling armatures in the BGE like you can read `here <http://lists.blender.org/pipermail/robotics/2009-September/000114.html>`_.

The individual bones in the BGE (Blender Game Engine) are called `'channels' <http://www.blender.org/documentation/blender_python_api_2_57_release/bge.types.html#bge.types.BL_ArmatureChannel>`_.
You can get these channels trough the BGE armature object, with `BL_ArmatureObject.channels <http://www.blender.org/documentation/blender_python_api_2_58_release/bge.types.html#bge.types.BL_ArmatureObject.channels>`_.
You can read and write the joint rotation values with the `BL_ArmatureChannel.joint_rotation <http://www.blender.org/documentation/blender_python_api_2_57_release/bge.types.html#bge.types.BL_ArmatureChannel.joint_rotation>`_ attribute [4].
Don't forget to call `BL_ArmatureObject.update() <http://www.blender.org/documentation/blender_python_api_2_58_release/bge.types.html#bge.types.BL_ArmatureObject.update>`_ after you have set the joint rotation.

IK limits will be enforced by blender, so the rotation values you set can differ from the actual rotation values that are set. You can edit the IK limits if you select the armature in *pose mode* and go to: *Properties* panel -> *Bone* -> *Inverse Kinematics*.

If you have an active IK constraint on a bone, be sure to disable this by removing the target, otherwise the constrain will reset the rotation values.

