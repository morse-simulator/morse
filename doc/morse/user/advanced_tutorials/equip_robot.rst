Building an equipped robot 
==========================

This tutorial provides instructions on how to create a file for a robot
prepared with a defined number of components, which can later be inserted into
scenario files.

Setup of the robot file 
-----------------------

Open Blender and erase all objects in the file:

#. Press :kbd:`a` to select all
#. Press :kbd:`x` and :kbd:`enter` to delete

Save the file with a name that represents the settings of the robot. As an
example this document will use the Ressac helicopter, so the name of the file
should be something like:
``$ORS_ROOT/share/data/morse/components/robots/ressac_equiped.blend``

#. Press :kbd:`F2` to open the ``Save as`` dialog
#. Navigate to the correct path and type the name of the file
#. Press the **Save File** button

Next link in the base of the robot from the component library:

#. With the mouse over the 3D view in Blender, press :kbd:`Shift-F1` to open the Load Library browser
#. Navigate to the directory ``$ORS_ROOT/data/morse/components/robots``
#. Press :kbd:`Left Mouse Click` over the file ``ressac.blend``
#. Press :kbd:`Left Mouse Click` over the item ``Object``
#. Toggle the buttons **Relative Paths** and **Link** at the bottom of the window
#. Press :kbd:`Right Mouse Click` and drag over the names of all the objects listed, to select them all
#. Press the button **Load Library**. You'll return to the 3D View
#. Select the newly inserted objects in the scene, either by 
   :kbd:`Right Mouse Click` clicking over the ``Ressac`` object in the 3D View, or 
   :kbd:`Left Mouse Click` over the object's name in the Outliner window. The object 
   will be highlighted in cyan colour, and can not be moved around.
#. Select the child objects, by pressing :kbd:`Shift-g`, then hitting :kbd:`enter`
#. Convert the objects to local, by pressing :kbd:`l` then hitting :kbd:`enter`

The rest of the components (sensors and actuators) should be linked similarly. Refer to the :doc:`Quick tutorial <../user/tutorial>` for instructions. In the case of a robot file, no middlewares or modifiers should be added, since those would be specific to every particular scenario.

Adjust the properties of the component if necessary. Then save the file again, by pressing :kbd:`Ctrl-w`, followed by :kbd:`enter`.

This robot file should be liked into scenarii files by following the same procedure, while selecting all the objects contained in the file.
