MOOS
====

Installation
------------

MOOS and pymoos must both be installed in order to use the MOOS middleware. 

Please follow the instructions in the :doc:`installation procedure  <../installation>`.


- Python: ``$MORSE_ROOT/src/morse/modifiers/moos_datastream.py``

Generation of MOOS app and variables
------------------------------------

The MOOS middleware creates a MOOS application called "MORSE_SIM" and posts a
message to the MOOS database for each variable output by a sensor. 
The names of the MOOS database variables are generated in the following way:

``<name_of_parent_blender_object>_<name_of_blender_object>_<variable_name>``

.. _moos_ds_configuration:

Configuration specificities
---------------------------

There is no special configuration parameter for the MOOS middleware.
