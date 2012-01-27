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

Linking a modifier to a component
---------------------------------

Binding a component to use a modifier is done in the file
``component_config.py`` that should be part of every MORSE scenario file. In
that file, the dictionary ``component_modifier`` lists the components and the
modifiers they will use to export/import their data. The unique names of the
components are the keys of the dictionary, and the values are lists. Each list
will contain as many items as modifiers associated to the component, and each
element is also a list. In these internal lists, the first element is the fully
qualified class name of the modifier, and the second is the modifier
method that should be called by the component to alter its data. Here is an
example of the ``component_modifier`` dictionary::

  component_modifier = {
    "GPS": [ ["morse.modifiers.ned.MorseNEDClass", "blender_to_ned"], ["morse.modifiers.utm.MorseUTMClass", "blender_to_utm"] ],
    "Motion_Controller": [ ["morse.modifiers.ned.MorseNEDClass", "ned_to_blender"], ["morse.modifiers.utm.MorseUTMClass", "utm_to_blender"] ],
  }

Creating a new modifier 
-----------------------

Please refer to the developer documentation: :doc:`Creating a modifier <../dev/adding_modifier>`.
