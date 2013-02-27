Integrate MORSE in your software architecture
=============================================

The core of MORSE is pure and independent of any specific robotic
implementation. To make it possible to integrate easily with your architecture,
MORSE proposes a *middleware* layers which bridge this gap, both for services
with the concept of :doc:`overlays` and for datastream with the concept of
`datastream handler` (this page).

.. _compatibility-matrix:

Features compatibility matrix
-----------------------------

The table below summarizes the level of support of MORSE features for each middleware.
Further information can be found in clicking on the desired component.

When a component is supported (✔), we specify the middleware-specific format
we use (like ``Viam`` or ``YarpImage``). 

If no format is specified, MORSE uses a generic serialization mechanism, e.g.,
a straight serialization of the JSON representation of the component data. 

.. csv-table:: 
    :header-rows: 1
    :stub-columns: 1
    :file: compatibility_matrix.csv


Configuring Morse to export data in a specified way
---------------------------------------------------

To configure a component to export or import data using a specific method, you
need to call the
:py:meth:`morse.builder.abstractcomponent.AbstractComponent.add_stream` on
your component. The process is explained in more details and with numerous
examples in the following tutorials:
	
- :doc:`beginner_tutorials/tutorial`
- :doc:`beginner_tutorials/yarp_tutorial`
- :doc:`beginner_tutorials/ros_tutorial`


Extending middleware support of MORSE
-------------------------------------

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
these middlewares (that the main purpose of the middleware layer after all!).
