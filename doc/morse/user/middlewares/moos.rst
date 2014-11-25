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

When configuring a component to export its data through MOOS, you can pass
the options ``moos_host`` and ``moos_port`` to define the host and port of
the MOOS community in which you want to communicate. Default values are 
``127.0.0.1:9000``.


.. code-block :: python

    foo.add_stream('moos', moos_port=9002, moos_host="127.0.0.2")


The same way, the option ``moos_freq`` defines the frequency with which 
data should be exported. The default value is 10Hz.

.. code-block :: python

    foo.add_stream('moos', moos_freq=20)
