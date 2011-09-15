Creation of new middlewares
===========================

The Python script
-----------------

- Copy one of the existing Python files in ``$MORSE_ROOT/src/morse/middleware``
- Change the name of the class, and write down its behaviour, implementing
  the ``register_component`` function

If you have a few different messages to decode / encode, it is doable to
implement everything in the single file ``${middleware}_mw.py``. However, if you
have a lot of different message to encode / decode, it is preferable to split
each processing in separate file. For this purpose, you need to create a
directory ``${middleware}``. Each file will implement a special processing for
some message. These files need to implement at least two different methods :

- ``init_extra_module(self, component_instance, function, mw_data)`` : this
  method will register the new serializer to its associated middleware
- a method to really serialize / deserialize the component input / output.

When you write a method which serialize the component output, you must not
modify the associated component. You are allowed to make computation to adapt
internal Morse structure to the need of specific middleware (it is the main
goal of this layer in fact). If you feel that some sensor / actuator requires
more information, you must add (propose a patch, or inherit and extend) it at
the sensor level and not at the middleware level, so every potential users
can profit from it (and not only people using this specific middleware).

The Blender file
----------------

- Copy one of the existing Blender files in
  ``$MORSE_ROOT/data/morse/middleware``
- Change the name of the ``Empty`` object. This name will be used to identify
  the middleware in the file ``component_config.py`` that binds components
  with middlewares

  - Press :kbd:`n` over Blender's 3D View window. This will display the
    **Transform Properties** of the object.
  - The name of the object is shown at the top left of this new window, and 
    can be changed there

- Change the Logic Properties of the Empty

  - Press :kbd:`F4` to display the Logic buttons
  - Set the **Class** variable to be equal to the name of the Python class
    previously defined
  - Set the **Path** variable with the path and name of the Python script. The
    path is relative to the ``morse/src`` directory in the installation
