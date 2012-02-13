Building an equipped robot
==========================

This tutorial provides instructions on how to create a Blender file for a robot
prepared with a defined number of components, which can later be inserted into
scenario files.

.. warning::
    The preferred way of creating equipped robots is now using the
    ``Builder API``. See the documentation on how to use it.


Setup of the robot file 
-----------------------

Launch MORSE by typing ``morse``, and erase all objects in the file:

#. Press :kbd:`a` to select all
#. Press :kbd:`x` and :kbd:`enter` to delete

Save the file with a name that represents the settings of the robot. As an
example this document will use the Ressac helicopter, so the name of the file
should be something like:
``$MORSE_ROOT/share/morse/data/robots/myrobot_equiped.blend``

#. Press :kbd:`F2` to open the ``Save as`` dialog
#. Navigate to the correct path and type the name of the file
#. Press the **Save File** button

Next link in the base of the robot from the component library:

#. With the mouse over the 3D view in Blender, press :kbd:`Ctrl-Alt-O` to open the Load Library browser
#. Navigate to the directory ``$MORSE_ROOT/morse/data/robots``
#. Press :kbd:`Left Mouse Click` over the file ``ressac.blend``
#. Press :kbd:`Left Mouse Click` over the item ``Object``
#. Press :kbd:`Right Mouse Click` and drag over the names of all the objects listed, to select them all
#. Press the button **Link/Append from Library**. You'll return to the 3D View, and the newly added
   human is selected, but can not move around.
#. Convert the objects to local: without de-selecting the object, press :kbd:`l` then hit :kbd:`enter`
#. If you deselected the inserted objects in the scene, select it again either by 
   :kbd:`Right Mouse Click` clicking over the object in the 3D View, or 
   :kbd:`Left Mouse Click` over the object's name in the **Outliner** panel. The object 
   will be highlighted in cyan colour.
#. Select as well the child objects, by pressing :kbd:`Shift-g`, then hitting :kbd:`enter`

The rest of the components (sensors and actuators) should be linked similarly.
Refer to the tutorial about :doc:`creating a scene in Blender
<../advanced_tutorials/editing_in_blender>` for instructions.
In the case of a robot file, no middlewares or modifiers should be added,
since those would be specific to every particular scenario.

Adjust the properties of the component if necessary. Then save the file again,
by pressing :kbd:`Ctrl-w`, followed by :kbd:`enter`.

This robot file should be liked into scenarii files by following the same
procedure, while selecting all the objects contained in the file.
