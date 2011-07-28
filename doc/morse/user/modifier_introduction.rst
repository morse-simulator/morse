Modifiers 
=========

Modifiers affect directly the data employed by sensors and actuators, and are
specific to the data used by the components. Just like middlewares, they must
implement a method called ``register_component`` that should add the
corresponding function to the component's action list.

List of existing modifiers 
--------------------------

.. toctree::
    :glob:
    :maxdepth: 1

    modifiers/*

Linking a modifier in a scene 
-----------------------------

To be able to use a modifier inside of a scene, it is necessary to link the
Empty object from the corresponding Blender file. This process is identical to
the one used for middlewares, as explained in the :doc:`basic tutorial <tutorial>`, together with an explanation on how to configure the components to
call middleware functions.

Binding a component to use a modifier is done in the file
``component_config.py`` that should be part of every MORSE scenario file. In
that file, the dictionary ``component_modifier`` lists the components and the
modifiers they will use to export/import their data. The unique names of the
components are the keys of the dictionary, and the values are lists. Each list
will contain as many items as modifiers associated to the component, and each
element is also a list. In these internal lists, the first element is the name
of the modifier Empty object in the scene, and the second is the modifier
method that should be called by the component to alter its data. Here is an
example of the ``component_modifier`` dictionary::

  component_modifier = {
    "GPS": [ ["NED", "blender_to_ned"], ["UTM", "blender_to_utm"] ],
    "Motion_Controller": [ ["NED", "ned_to_blender"], ["UTM", "utm_to_blender"] ],
  }

Creating a new modifier 
-----------------------

Please refer to the developer documentation: :doc:`Creating a modifier <../dev/adding_modifier>`.
