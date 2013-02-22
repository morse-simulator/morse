How to efficiently add complex bounding boxes to your objects 
=============================================================

It is often important to simplify the bounding box of complex objects (ie,
replace a complex ``Triangle Mesh`` by a set of simpler boxes or spheres for
instance) to maintain the performances of the MORSE physics engine (Bullet).

Here is the "how-to".

Let's consider your object is named for instance ``RollingChair``.

.. image:: ../../../media/object_grouping.png
  :align: center

#. Append a ``_mesh`` suffix to its name (for instance ``RollingChair_mesh``).
#. In the Physics properties panel, disable the physics for this object (``No Collision``).
#. Create plain boxes and put them around your object to shape the bounds. Name
   them after your object, appending a suffix (like ``RollingChair_bb_xx``, cf
   image above).
#. Apply scaling (:kbd:`Ctrl-a`) on each boxes.
#. Create a new ``Empty`` and name it after your object (in our example
   ``RollingChair``). Place the empty at the origin of your object (it will
   correspond to the position of the object in MORSE).
#. Parent all bounding boxes + the original object mesh to this empty (select
   all object, ending with the empty, and press :kbd:`Shift+M`).
#. In the Physics properties, set the empty to be a ``RigidBody``. Set here as
   well properties like the mass and the translation/rotation damping.
#. Set the physics properties for all other boxes **belonging to the bounding
   box** to ``RigidBody``, not forgetting to set the radius to be smaller than
   the box. All the boxes must have their ``Collision bounds`` set to ``Box``
   and ``Compound`` must be selected. Let other physics properties to their
   defaults.
#. All the bounding boxes can be set as well to ``Invisible`` in the same
   Physics panel.

Check the :doc:`passive objects<../others/passive_objects>` doc for details on
creation of objects in general.
