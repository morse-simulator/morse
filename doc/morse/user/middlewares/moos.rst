MOOS
====

Installation
------------

MOOS and pymoos must both be installed in order to use the MOOS middleware.

Please follow the instructions in the :doc:`installation procedure  <../installation>`.


Generation of MOOS app and variables
------------------------------------

The MOOS middleware creates a MOOS application called "iMorse" and posts a
message to the MOOS database for each variable output by a sensor.
The names of the MOOS variables all start with ``MORSE_*``.


Configuration specificities
---------------------------

When configuring a component to export its data through MOOS, you can pass
the options ``moos_host`` and ``moos_port`` to define the host and port of
the MOOS community in which you want to communicate. Default values are
``127.0.0.1:9000``.
If for a specific component you don't want to pass through the default
``iMorse_default`` app name, you can specify an new name with ``moos_name``.


.. code-block :: python

    foo.add_stream('moos', moos_port=9002, moos_host="127.0.0.2")
    bar.add_stream('moos', moos_name='iMorse_bar')
