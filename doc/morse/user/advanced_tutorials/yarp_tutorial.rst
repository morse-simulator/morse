YARP-based simulation tutorial
==============================

This tutorial shows a simple scenario with Yarp: Simple dummy autonomous navigation towards a user-given target (x,y). 
The robots becomes red when it intersects obstacles or bounces on them.

Setup
-----

You need to install YARP and its Python bindings, by following the YARP section in the :doc:`installation notes <../installation>`.

Before running a simulation using YARP, it is necessary to open a new shell terminal and start the ``yarpserver`` program::

  $ yarpserver

Configuring the scenario
------------------------

You must link a YARP middleware object into the MORSE scenario file
Create the bindings of the components with yarp, by editing the file ``component_config.py`` inside the Blender file.


Link a Camera sensor
++++++++++++++++++++

#. With the mouse over the 3D view in Blender, press :kbd:`Shift-F1` to open the Load Library browser
#. Navigate to the directory ``$MORSE_ROOT/data/morse/sensors``
#. Press :kbd:`Left Mouse Click` over the file ``morse_camera.blend``
#. Press :kbd:`Left Mouse Click` over the item ``Object``
#. Toggle the buttons **Relative Paths** and **Link** at the bottom of the window
#. Press :kbd:`Right Mouse Click` over the items ``CameraMain``, ``CameraUser``, ``CameraCube``, ``CameraLens``
#. Press the button **Load Library**. You'll return to the 3D View
#. Select the newly inserted ``CameraMain`` object in the scene, either by
   :kbd:`Right Mouse Click` clicking over the object in the 3D View, or :kbd:`Left
   Mouse Click` over the object's name in the Outliner window. The object will be
   highlighted in cyan colour, and can not be moved around.  #. Select the child
   object, by pressing :kbd:`Shift-g`, then hitting :kbd:`enter`
#. Convert the object to local, by pressing :kbd:`l` then hitting :kbd:`enter`
#. Switch to front view by pressing :kbd:`1`
#. Press :kbd:`g`, then move the ``CameraMain`` object to the correct location with respect to the robot
#. Press :kbd:`Left Mouse Click` to accept the movement
#. With the ``CameraMain`` object selected, hold down :kbd:`S` and then :kbd:`Right Mouse Click` over the robot object
#. Press :kbd:`Ctrl-p` and then hit :kbd:`enter` make the robot the parent of the controller

Insert the middleware object
++++++++++++++++++++++++++++

#. With the mouse over the 3D view in Blender, press :kbd:`Shift-F1` to open the Load Library browser
#. Navigate to the directory ``$MORSE_ROOT/data/morse/middleware``
#. Press :kbd:`Left Mouse Click` over the file ``yarp_mw.blend``
#. Press :kbd:`Left Mouse Click` over the item ``Object``
#. Toggle the buttons **Relative Paths** and **Link** at the bottom of the window
#. Press :kbd:`Right Mouse Click` over the item ``Yarp_Empty``
#. Press the button **Load Library**. You'll return to the 3D View
#. It is not necessary to make this object local or to move it. But it can be useful to avoid cluttering of items in the scene 

.. note:: One single middleware Empty is necessary to enable the middleware, regardless of how many components will make use of it.

Configuring the middlewares
+++++++++++++++++++++++++++

Binding the components in the scene with the middleware is done in a configuration file within the Blender file.

#. On the **Text Editor** window, select the file ``component_config.py``
#. Add the following items to the ``component_mw`` dictionary::
  
    component_mw = {
        "CameraMain": [["Yarp", "post_image_RGBA"]],
        "GPS": [["Yarp", "post_message"]],
        "Motion_Controller": [["Yarp", "read_message"]],
    }
  

Reading/writing data
--------------------

When the simulation starts, it will print the names of the YARP ports that have
been created for every corresponding component. These port names can be used to
connect to the component from an external program or client.

The simplest method to test the reading and writing of data is by using the
terminal clients. For example, to read the GPS data of the robot through a port
named ``/ors/robots/ATRV/GPS/out``, you can type the following in a
terminal::

  $ yarp read /data/in /ors/robots/ATRV/GPS/out

To enter speed commands through a port named ``/ors/robots/ATRV/Motion_Controller/in``, use the command::

  $ yarp write /data/out /ors/robots/ATRV/Motion_Controller/in

Then type the three destination coordinates, separated by spaces, and press :kbd:`enter`

To view the images of the camera though a port ``/ors/robots/ATRV/CameraMain/out``::

  $ yarpview /img/read &
  $ yarp connect /ors/robots/ATRV/CameraMain/out /img/read
