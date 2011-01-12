The MORSE user workflow 
=======================

The general steps to follow to do a simulation in MORSE are the following:

#. Create the :doc:`robot configuration <user/advanced_tutorials/equip_robot>` that matches the real robot
#. Create a scenario file, based on the default MORSE file, using::

  $ morse create [file_name]

#. Save the new scene :kdb:`F2` and enter the name
#. Link in the robot
#. Link in any additional robots/sensors/actuators
#. Link in the middleware and modifier objects
#. Configure the components to use specific modifiers and middlewares
#. Prepare the middleware environments (in the case of YARP or Pocolibs)
#. Save the file before starting the simulation :kdb:`Ctrl-w`
#. Start the simulation :kdb:`p`
#. Look at the terminal window to check that everything started correctly
#. Start the client programs that will connect with the simulator
#. Finish the simulation by pressing :kdb:`esc`
#. Quit Blender by pressing :kdb:`Ctrl-q` and :kdb:`enter`
