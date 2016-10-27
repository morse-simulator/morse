Understanding Morse's code organisation
=======================================

Morse's source code is organised in the following way :

- ``addons``: this contains various addons for blender, useful for MORSE simulation
- ``bin``: this contains the main entry point for the MORSE simulation
- ``bindings``: this contains some Python libraries for accessing MORSE data, through
  the socket middleware
- ``config``: this contains things needed for CMake 
- ``data``: this contains the blender model of sensors / actuators
- ``doc``: the documentation (in reStructuredText format)
- ``examples``: this contains examples that show :

  - how to control the simulator (in ``clients``)
  - various test scenarios (in ``scenarii``)
- ``src``: this contains all the Python scripts used by the simulator : it is the
  simulator's core

  - ``morse/actuators``: this contains some predefined robot actuators
  - ``morse/blender``: this contains some scripts needed for the initialization of
    the game engine
  - ``morse/builder``: this contains the scripts for the API that permits creating a
    simulation scenario from a Python file. See the :doc:`Builder API
    <../user/builder>` documentation.
  - ``morse/core``: this contains core classes for the MORSE project (services, base
    objects, sensors, etc.) 
  - ``morse/helpers``: this contains various helper functions (e.g., math transformations)
  - ``morse/middleware``: this contains the code for linking with different
    middlewares, both for services and datastreams. Each middleware has its own separate
    subdirectory.
  - ``morse/modifiers``: this contains implementations of various modifiers for basic
    components
  - ``morse/robots``: this contains some predefined robot classes
  - ``morse/sensors``: this contains some predefined robot sensors

- ``testing``: this contains unit tests for various part of MORSE.
