Understanding Morse code organisation
=====================================

The source code of Morse is organised in the following way :

- ``addons``: it contains various addons for blender, useful for MORSE simulation
- ``bin``: it contains the main entry point of the MORSE simulation
- ``bindings``: it contains some python library to access to MORSE data, through
  the socket middleware
- ``config``: it contains stuff for Cmake 
- ``data``: it contains the blender model of sensors / actuators
- ``doc``: the documentation (in rest format)
- ``examples``: it contains examples about :

  - how to control the simulator (in ``clients``)
  - more or less elaborate test scenarii (in ``scenarii``)
- ``src``: it contains all the python scripts used by the simulator : it is the
  core of the simulator

  - ``morse/actuators``: it contains implementation for various robot actuators
  - ``morse/blender``: it contains some scripts needed at the initialization of
    the game engine
  - ``morse/builder``: it contains the scripts for the API that permits creating a
    simulation scenario from a Python file. See the :doc:`Builder API
    <../user/builder>` documentation.
  - ``morse/core``: it contains core classes for the MORSE project (services, base
    objects, sensors, ...) 
  - ``morse/helpers``: it contains various helpers (math transformation)
  - ``morse/middleware``: it contains the code for linking with different
    middlewares, both for service or datastream. Each middleware has a separate
    directory.
  - ``morse/modifiers``: it contains implementation for various modifiers to basic
    components
  - ``morse/robots``: it contains instantiation of different robot classes
  - ``morse/sensors``: it contains implementation for various robot sensors

- ``testing``: it contains "unit-test" for various part of MORSE.
