Property sensor
==========

This sensor reads properties values of robot. Properties names are given in
NamesOfProperties property (as comma separated list). If no property name is given sensor will read all
robot properties.

Files
-----
- Blender: ``$MORSE_ROOT/data/sensors/property.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/property.py``

Local data
~~~~~~~~~~
Number and name of variables are defined with number/names of properties.

Properties
----------

  - **NamesOfProperties**: (string) Comma separated list of property names.

Applicable modifiers 
--------------------

No available modifiers
