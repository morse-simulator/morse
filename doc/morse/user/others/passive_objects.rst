Interacting with passive objects
================================

Besides physical interaction and perception via cameras or depth sensors,
passive objects in a simulation can be interacted with in several other ways.

For instance some objects can be set to be graspable by a robot (or a human), 
and specific sensors like the :doc:`Semantic camera <../sensors/semantic_camera>` 
may provide extended facts on a particular object, like its type.

To be set as an **interactive** passive object, you only have to add the (Game)
property ``Object`` to the object, and to set it as a ``True`` boolean property.

Other, **optional**, properties allow to control the behaviour of the object:

- ``Label`` (``String``): the name (or label) of the object [#]_,
- ``Description`` (``String``): a longer description of the object [#]_,
- ``Type`` (``String``): the type of the object [#]_,
- ``Graspable`` (``Boolean``): if the object is graspable or not [#]_.

You can temporarly disable an object by simply setting its ``Object`` property to false.

.. [#] Used to display the object name in :doc:`human's manipulation mode <human>`
   and by the semantic camera sensor.
.. [#] Not used yet.
.. [#] Used by the semantic camera sensor, defaults to ``Object``.
.. [#] Used by the human's manipulation mode and the :doc:`gripper <../actuators/gripper>` 
   sensors.

.. note::
   
   For the manipulation routines to work, the above properties (especially, ``Graspable``)
   must be set on the **object holding the mesh you want to grab**.
   
   It is often convenient to have the object to be manipulated as an empty, with its mesh
   **as parent**. In that specific case (the mesh is a direct parent of the object that 
   holds the interaction properties), your can safely put the ``Graspable`` property on the
   empty. MORSE will automatically select the parent mesh when it must be grasped.

.. note::
  
   You can also have a look to the tips to add a :doc:`bounding box around your objects 
   <../tips/bounding_boxes>`.
