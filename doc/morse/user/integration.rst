
Integrate MORSE in your software architecture
=============================================

By relying on various *middlewares* as the primary mean of communication with
other software modules, MORSE should be able to insert in your existing
architecture with minimum changes.

We also provide (partial) language bindings that allows to directly (at the programming language level) include support MORSE in other softwares.

Bindings
++++++++

Currently, we only have bindings for the Python language (contributions are welcome!).

These bindings are documented here: :doc:`pymorse <../pymorse>`.

Middlewares support
+++++++++++++++++++

Middlewares provide a means for the simulated data to be shared with external
programs. MORSE is designed to be middleware independent, so that its internal
functioning is not tied no any one particular middleware, but it is capable of
communicating with any type of architecture.

The binding of middlewares with the components in the scene is done using
the concept of :doc:`hooks <hooks>`.

Current list of compatible middlewares 
--------------------------------------

.. toctree::
    :glob:
    :maxdepth: 1

    middlewares/*

Refer to the table below for details regarding which features are actually
supported for each middleware.

.. _compatibility-matrix:

Features compatibility matrix
-----------------------------

The table below summarizes the level of support of MORSE features for each middleware.

When a component is supported (✔), we specify the middleware-specific format we
use (like ``Viam`` or ``YarpImage``). 

If no format is specified, MORSE uses its default serialization mechanism,
i.e., a straight serialization of the JSON representation of the component
data. Please refer to each middleware to know more about it.

.. csv-table:: 
    :header-rows: 1
    :stub-columns: 1
    :file: compatibility_matrix.csv


Linking a middleware in a scene 
-------------------------------

It is no longer necessary to link another object into the scene to get access
to middleware connectivity.
In the current version of MORSE, it is enough to properly describe the
middleware that will be used by each component.

The simplest way to configure the middlewares is by using the Builder API.
This process is explained in the
:doc:`basic tutorial <beginner_tutorials/tutorial>` and the
:doc:`yarp tutorial <beginner_tutorials/yarp_tutorial>`.

Alternatively, when building a scene using the Blender interface, as explained
in the :doc:`tutorial <advanced_tutorials/editing_in_blender>`, you have to\
add the full description of the middleware into the file ``component_config.py``
that should be part of every MORSE scenario file.
In that file, the dictionary ``component_mw`` lists the components and the
middleware they will use to export/import their data. The unique names of the
components are the keys of the dictionary, and the values are lists.
The first item in the list is the full path and class name of the middleware
you wish to use, followed by the name of the middleware function that should
be called by the component to share its data.
Additional parameters may be necessary to use specific bindings between
particular components and middlewares.
More information about the format of this file can be found in the
:doc:`hooks <hooks>` documentation.

Adding support for new middlewares 
-----------------------------------

New middlewares can be added to MORSE by following these 
:doc:`instructions <../dev/new_middleware>`.
