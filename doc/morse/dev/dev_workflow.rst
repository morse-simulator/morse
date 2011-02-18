The MORSE workflow for developers
=================================

The internal functioning of MORSE is based on Blender's Game Engine events.
These are defined in the Logic buttons window, using the graphical interface
called "Logic Bricks". These are a set of predefined **Sensor**, **Controller**
and **Actuator** events which can be linked together.  **Actuators** in
particular are important because they are the places where Python scripts are
called.

For Blender to find the Python scripts refered to in the Actuators, the Python
files must be directly in a directory listed in ``PYTHONPATH``. The standard
location for these files in MORSE is the directory:
``MORSE_ROOT/src/morse/blender``, which should be included in ``PYTHONPATH`` as
described in the :doc:`MORSE installation documentation <../user/installation>`.

Each object in Blender can have its own set of Logic Bricks. In every MORSE
simulatino scene, there must be one ``Scene_Script_Holder`` object, which holds
the predefined Logic Bricks necessary to initialize and control the simulation.
When a simulation is started (when launching the Game Engine) it will call the
initialization scripts of MORSE, contained in the file ``$MORSE_ROOT/src/morse/blender/main.py`` .

The script ``main.py`` is charged of multiple tasks:

-  Upon lanching the simulation, it will initialize all components:
    -  Create a dictionary of robots
    -  Create a dictionary of components, and the robot their are associated with
    -  Create the dictionary of modifiers
    -  Create the dictionary of middlewares
    -  Link the modifiers and middlewares to components, as specified in the file ``component_config.py``
-  When the simulation ends, it will destroy the objects created, and call the
   methods to cleanup ports, files, connections, etc.

The initialization of all components, including middlewares and modifiers, is
done by instantiating an object of the Python class specific to every
component. This is done dynamically, using the **Class** and **Path**
properties that should be present in the Blender file of every component.

Events during a simulation
--------------------------

Blender's Game Engine is set to work at a predefined ``Tickrate``, which is the
number of times the Logic Brick events are executed per second. In the default
settings, it is equal to 60, so that during one real second there will be 60
"ticks".

While the simulation is running, the Logic Bricks of each component will make
regular calls to their ``default_action`` method. At this point the component
will perform its task and update its internal data.

Time management
---------------

The simulator will measure the amount of real time that has passed since the
Game Engine was started. Time is measured in seconds, and stored as a floating
point value with two decimals. It is obtained using the Python method:
``time.clock()``, which relies on the C function of the same name.
Simulation time is accessible from any Python script that imports the
``GameLogic`` module, by reading the variable: ``GameLogic.current_time``.
