The MORSE user workflow 
=======================

The general steps to follow to do a simulation in MORSE are the following:

#. Write a Python script with the description of the robot and scenario, using the :doc:`Builder API <../dev/builder>`. The file must specify:

    - The robots
    - The sensors and actuators
    - The middleware bindings
    - The environment properties

#. Execute the builder script with **morse**::
  
    $ morse edit [file_name].py
  
#. Save the new scene: Press :kbd:`F2` and enter a new name for the Blender file

#. Initialise the middleware environments (ROS, YARP, Pocolibs, etc.) outside of Blender

#. Start the simulation: Place your mouse inside the 3D view of Blender and press :kbd:`p`

#. Look at the terminal window to check that everything started correctly

#. Start the client programs that will connect with the simulator, and carry out your simulation experiment

#. Finish the simulation by pressing :kbd:`Esc`

#. Quit Blender by pressing :kbd:`Ctrl-q` and :kbd:`Enter`

#. To repeat the simulation using the same settings, run **morse** directly with the Blender file::

    $ morse edit [file_name].blend
