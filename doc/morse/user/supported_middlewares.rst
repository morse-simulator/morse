Middleware Support in MORSE
===========================

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

To be able to use a middleware inside of a scene, it is necessary to link the
Empty object from the corresponding Blender file. This process is explained in
the :doc:`basic tutorial <tutorial>` and the :doc:`yarp tutorial
<advanced_tutorials/yarp_tutorial>`. Those pages also explain how to
configure the components to use a given middleware.

Binding a component to use a middleware is done in the file
``component_config.py`` that should be part of every MORSE scenario file. In
that file, the dictionary ``component_mw`` lists the components and the
middleware they will use to export/import their data. The unique names of the
components are the keys of the dictionary, and the values are lists. The first
item in the list is the name of the middleware Empty object in the scene. The
following items depend on the type of middleware, but will generally be the
name of the middleware function that should be called by the component to share
its data.

Adding support for new middlewares 
-----------------------------------

New middlewares can be added to MORSE by following these 
:doc:`instructions <../dev/new_middleware>`.
