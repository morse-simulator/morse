Builder Overview
================

Here is a quick overview of what you can do with the Builder API:

+-------------+---------------------------------------+---------------------------------+
|             |                                       |                                 |
| Component   | General                               | Middleware specific             |
|             |                                       |                                 |
+-------------+---------------------------------------+---------------------------------+
|             |                                       |                                 |
| Robot       | - location = (x,y,z) : absolute       | - add_default_interface('name') |
|             | - rotation_euler = (x,y,z) : absolute | - make_external()               |
|             | - scale = (x,y,z)                     |                                 |
+-------------+ - translate(x,y,z) : relative         +---------------------------------+
|             | - rotate(x,y,z) : relative            |                                 |
| Sensor      | - append(component)                   | - add_stream('name')            |
|             | - frequency(hz)                       | - add_service('name')           |
+-------------+ - level(level)                        | - add_interface('name')         |
|             | - properties()                        |                                 |
| Actuator    | - alter(modifier)                     |                                 |
|             |                                       |                                 |
+-------------+---------------------------------------+---------------------------------+
|             |                                                                         |
| Environment | see :py:class:`morse.builder.environment.Environment`                   |
|             |                                                                         |
+-------------+-------------------------------------------------------------------------+

For a full doc on the Builder API, see :doc:`Build your simulations <builder>`.

You will find detailed documentation in the :py:mod:`morse.builder` page.
