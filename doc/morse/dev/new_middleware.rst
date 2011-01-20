Creation of new middlewares
===========================

The Python script
-----------------

-  Copy one of the existing Python files in ``$ORS_ROOT/src/morse/middleware``
-  Change the name of the class, and write down its behaviour, implementing the ``register_component`` function

The Blender file
----------------

- Copy one of the existing Blender files in ``$ORS_ROOT/data/morse/components/middleware``
- Change the name of the ``Empty`` object. This name will be used to identify the middleware in the file ``component_config.py`` that binds components with middlewares
  - Press :kbd:`n` over Blender's 3D View window. This will display the **Transform Properties** of the object.
  - The name of the object is shown at the top left of this new window, and can be changed there

- Change the Logic Properties of the Empty
  - Press :kbd:`F4` to display the Logic buttons
  - Set the **Class** variable to be equal to the name of the Python class previously defined
  - Set the **Path** variable with the path and name of the Python script. The path is relative to the ``morse/src`` directory in the installation
