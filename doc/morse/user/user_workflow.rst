The MORSE user workflow 
=======================

The general steps to follow to do a simulation in MORSE are the following:

#. Write a Python script with the description of the robot and scenario, using the Builder API. The file must specify:

    - The robots
    - The sensors and actuators
    - The middleware bindings
    - The environment properties

#. Execute the builder script with **morse**::
  
    $ morse exec [file_name].py
  
#. Save the new scene: Press :kbd:`F2` and enter a new name for the Blender file

#. Prepare the middleware environments (in the case of YARP or Pocolibs) outside of Blender

#. Start the simulation: Place your mouse inside the 3D view of Blender and press :kbd:`p`

#. Look at the terminal window to check that everything started correctly

#. Start the client programs that will connect with the simulator

#. Finish the simulation by pressing :kbd:`Esc`

#. Quit Blender by pressing :kbd:`Ctrl-q` and :kbd:`Enter`


#. To repeat the simulation using the same settings, run **morse** directly with the Blender file::

    $ morse [file_name].blend
