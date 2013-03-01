Builder Overview
================

Here is a quick overview of what you can do with the Builder API:

+-------------+-----------------------------+--------------------------------+
| Component   | General                     | Middleware specific            |
+-------------+-----------------------------+--------------------------------+
|             |                             |                                |
| Robot       | - location = (x,y,z)        | - add_default_interface        |
|             | - rotation_euler = (x,y,z)  | - make_external                |
|             | - scale = (x,y,z)           |                                |
+-------------+ - translate(x,y,z)          +--------------------------------+
|             | - rotate(x,y,z)             |                                |
| Sensor      | - append(component)         | - add_stream                   |
|             | - frequency(hz)             | - add_service                  |
+-------------+ - level(level)              | - add_interface                |
|             | - properties()              |                                |
| Actuator    | - alter(modifier)           |                                |
|             |                             |                                |
+-------------+-----------------------------+--------------------------------+
|             |                                                              |
| Environment | see :py:class:`morse.builder.environment.Environment`        |
|             |                                                              |
+-------------+-----------------------------+--------------------------------+

For a full doc on the Builder API, see :doc:`Build your simulations <builder>`.

You will find detailed documentation in the :py:mod:`morse.builder` page.
