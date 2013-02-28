Entry point of the simulation
=============================

The internal functioning of MORSE is based on Blender's Game Engine events.
These are defined in the Logic buttons window, using the graphical interface
called "Logic Bricks". These are a set of predefined **Sensor**, **Controller**
and **Actuator** events which can be linked together.  **Actuators** in
particular are important because they are the places where Python scripts are
called.

Each object in Blender can have its own set of Logic Bricks. In every MORSE
simulation scene, there must be one ``Scene_Script_Holder`` object, which holds
the predefined Logic Bricks necessary to initialize and control the simulation.
When a simulation is started (when launching the Game Engine) it will call the
initialization scripts of MORSE, contained in the file ``$MORSE_ROOT/src/morse/blender/main.py`` .

The script ``main.py`` is charged of multiple tasks:

-  Upon launching the simulation, it will initialize all components:
    -  Create a dictionary of robots
    -  Create a dictionary of components, and the robot there are associated with
    -  Create the dictionary of modifiers
    -  Create the dictionary of middlewares
    -  Create the dictionary of services
    -  Create the dictionary of overlays
    -  Link the modifiers, middlewares, services overlays to their respective
       components, as specified in the file ``component_config.py``

-  When the simulation ends, it will destroy the objects created, and call the
   methods to cleanup ports, files, connections, etc.

The initialization of all components, including middlewares and modifiers, is
done by instantiating an object of the Python class specific to every
component. This is done dynamically, using the **Class** and **Path**
properties that should be present in the Blender file of every component.
