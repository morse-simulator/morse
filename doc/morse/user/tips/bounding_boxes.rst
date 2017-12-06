How to efficiently add complex bounding boxes to your objects 
=============================================================

It is often important to simplify the bounding box of complex objects (i.e.,
replace a complex ``Triangle Mesh`` with a set of simpler boxes or spheres for
instance), to ensure good performance of the MORSE physics engine (Bullet).

Here is an example of how to simplify a bounding box.

Let's consider an object called, for example, ``RollingChair``.

.. image:: ../../../media/object_grouping.png
  :align: center

#. Append a ``_mesh`` suffix to its name (e.g., ``RollingChair_mesh``).
#. In the Physics properties panel, disable the physics for this object (``No Collision``).
#. Create plain boxes and put them around the object to shape the bounds. Name
   them after your object, appending a suffix (like ``RollingChair_bb_xx``,
   as shown in the image above).
#. Apply scaling (:kbd:`Ctrl-a`) on each box.
#. Create a new ``Empty`` and name it after the object (in our example
   ``RollingChair``). Place the empty at the origin of the object (it will
   correspond to the position of the object in MORSE).
#. Parent all bounding boxes and the original object mesh to this empty (select
   all the objects ending with the empty, and press :kbd:`Shift+M`).
#. In the Physics properties, set the empty to be a ``RigidBody``. Also
   set its other properties like its mass and translation/rotation damping.
#. Set the physics properties for all other boxes **belonging to the bounding
   box** to ``RigidBody``, not forgetting to set the radius to be smaller than
   the box. All the boxes must have their ``Collision bounds`` set to ``Box``
   and ``Compound`` must be selected. Leave the other physics properties to their
   defaults.
#. All the bounding boxes can also be set to ``Invisible`` in the same
   Physics panel.

Check the :doc:`passive objects<../others/passive_objects>` doc for details on
the creation of objects in general.
