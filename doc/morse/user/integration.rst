Integrating MORSE into your software architecture
=================================================

MORSE's core is independent of any specific robotic implementation. To
make it possible to easily integrate with your particular robotic architecture,
MORSE provides *middleware* layers which bridge the gap, both for
services with the concept of :doc:`overlays` and for datastreams with the
concept of `datastream handlers` (this page).

.. _compatibility-matrix:

Features compatibility matrix
-----------------------------

The table below summarizes the level of support of MORSE features for each middleware.
Further information can be found by clicking on the relevant component.

When a component is supported (✔), we specify the middleware-specific format
MORSE uses (e.g., ``Viam`` or ``YarpImage``). 

If no format is specified, MORSE uses a generic serialization mechanism, e.g.,
a straight serialization of the JSON representation of the component data. 

.. csv-table:: 
    :header-rows: 1
    :stub-columns: 1
    :file: compatibility_matrix.csv


Configuring Morse to handle data in a specified way
---------------------------------------------------

To configure a component to export or import data using a specific method, you
need to call
:py:meth:`morse.builder.abstractcomponent.AbstractComponent.add_stream`
method for your component. This is explained in detail and with examples
in the following tutorials:
	
- :doc:`beginner_tutorials/tutorial` -
  :doc:`beginner_tutorials/yarp_tutorial` -
  :doc:`beginner_tutorials/ros_tutorial` -
  :doc:`advanced_tutorials/hla_tutorial`


Extending MORSE's middleware support
------------------------------------

The following :doc:`documentation <../dev/adding_datastream_handler>` explains
how to add a specific datastream handler for one existing middleware.
If you want to add a completely new middleware, you can refer to these
:doc:`instructions <../dev/new_middleware>`.

Bindings
--------

We provide some bindings to access the different components using the
:tag:`socket` interface and to control the simulation. At the moment, we only
have bindings for the Python language but contributions are welcome! These
bindings are documented :doc:`here <../pymorse>`.

For other middlewares, you can, of course, rely on standard tools provided by
these middlewares (that's the main purpose of the middleware layer after all!).
