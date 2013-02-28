Entry points of the simulation
==============================

The internal functioning of MORSE is based on Blender's Game Engine events.
These are defined in the Logic buttons window, using the graphical interface
called "Logic Bricks". These are a set of predefined **Sensor**, **Controller**
and **Actuator** events which can be linked together.  **Actuators** in
particular are important because they are the places where Python scripts are
called.

Each object in Blender have its own set of Logic Bricks. The entry point of a
Morse scene is defined in the ``Scene_Script_Holder`` object which is available
in ``${MORSE_ROOT}/data/props/basics.blend`` (this object is automatically
added by the Builder API if it is not found in the simulation). This specific
object contains the entry point :py:meth:`morse.blender.main.init` and the
finalisation point :py:meth:`morse.blender.main.finish`. It contains also the
"Logic Bricks" to control cameras, reset the simulation, ...

Let's take a look at the module :py:mod:`morse.blender.main`.

The init method
---------------

The :py:meth:`morse.blender.main.init` is responsible to initialize all the
Morse subsystems. It includes:

- Create the dictionary of robots (:py:data:`morse.core.blender.persistantstorage.robotDict`)
- Create the dictionary of components, and the robot there are associated
  with (:py:data:`morse.core.blender.persistantstorage.componentDict`)
- Create the dictionary of modifiers
  (:py:data:`morse.core.blender.persistantstorage.modifierDict`)
- Create the dictionary of datastream handlers
  (:py:data:`morse.core.blender.persistantstorage.datastreamDict`)
- Create the dictionary of services
  (:py:data:`morse.core.blender.persistantstorage.morse_services`)
- Create the dictionary of overlays
  (:py:data:`morse.core.blender.persistantstorage.overlayDict`)
- Link the modifiers, middlewares, services overlays to their respective
  components, as specified in the file ``component_config.py`` (see
  :doc:`arguments_passing`).
- Initialise the logging system.

The initialization of all components, including middlewares and modifiers, is
done by instantiating an object of the Python class specified in each
component by the variable **classpath**. This **classpath** is not stored in
blender file anymore, but described in the builder component (see
:py:mod:`morse.builder.sensors` for example).


The finalization method
-----------------------

The method which terminates the simulation (called when you press :kbd:`ESC`) is the
:py:meth:`morse.blender.main.finish`). However, the real cleaning logic is in
:py:meth:`morse.blender.main.close_all`, which try to properly finalize all
resources used by the simulator, in particular, resources used by the
different middlewares.

The main method
---------------

The ``Scene_Script_Holder`` defines too a method which it calls at each
simulator loop. It is the method :py:meth:`morse.blender.main.simulation_main`
which:

- update the simulation clock
- dispatch services if any
- synchronise with the multi-node master in multi-node case
