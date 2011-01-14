The MORSE user workflow 
=======================

The general steps to follow to do a simulation in MORSE are the following:

#. Create the :doc:`robot configuration <advanced_tutorials/equip_robot>` that matches the real robot
#. Create a scenario file, based on the default MORSE file, using::
  
    $ morse create [file_name]
  
#. Save the new scene: Press :kbd:`F2` and enter a new file name
#. Link in the robot
#. Link in any additional robots/sensors/actuators
#. Link in the middleware and modifier objects
#. Configure the components to use specific modifiers and middlewares: 
    Edit the ``component_config.py`` file in the Text Window
#. Prepare the middleware environments (in the case of YARP or Pocolibs) outside of Blender
#. Save the file before starting the simulation :kbd:`Ctrl-w`
#. Start the simulation :kbd:`p`
#. Look at the terminal window to check that everything started correctly
#. Start the client programs that will connect with the simulator
#. Finish the simulation by pressing :kbd:`Esc`
#. Quit Blender by pressing :kbd:`Ctrl-q` and :kbd:`Enter`
