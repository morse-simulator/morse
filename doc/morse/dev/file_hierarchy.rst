Understanding Morse's code organisation
=======================================

The Morse source code is organised in the following way :

- ``addons`` this contains various addons for Blender, useful for MORSE simulation
- ``bin`` this contains the MORSE simulation's main entry point
- ``bindings`` this contains some Python libraries to access to MORSE data, through
  the socket middleware
- ``config`` this contains support for CMake 
- ``data`` this contains the Blender model of sensors and actuators
- ``doc``: the documentation (in reStructuredText format)
- ``examples`` this contains examples about :

  - how to control the simulator (in ``clients``)
  - more or less elaborate test scenarios (in ``scenarii``)
- ``src`` this contains all the Python scripts used by the simulator; this is the
  core of the simulator

  - ``morse/actuators`` this contains implementations for various robot actuators
  - ``morse/blender`` this contains some scripts needed at the initialization of
    the game engine
  - ``morse/builder`` this contains the scripts for the API that permits creating a
    simulation scenario from a Python file. See the :doc:`Builder API
    <../user/builder>` documentation.
  - ``morse/core`` this contains core classes for the MORSE project (services, base
    objects, sensors, etc.) 
  - ``morse/helpers`` this contains various helper functions (e.g., math transformation)
  - ``morse/middleware`` this contains the code for linking with different
    middlewares, both for service or datastream. Each middleware has a separate
    directory.
  - ``morse/modifiers`` this contains implementations for various modifiers to basic
    components
  - ``morse/robots`` this contains implementations of various robot classes
  - ``morse/sensors`` this contains implementations of various robot sensors

- ``testing`` this contains unit tests for various parts of MORSE.
