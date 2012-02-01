Creation of new middlewares
===========================

The easiest way to add middleware communications to MORSE is to take a look
at the existing modules, copy one of them, and modify it according to
your requirements.

Probably the best example for this (at the same time simple and complete) is
the :doc:`YARP <../user/middlewares/yarp>` middleware.

The Python script
-----------------

- Copy one of the existing Python files in ``$MORSE_ROOT/src/morse/middleware``
- Change the name of the class, and write down its behaviour, implementing
  the ``register_component`` function

If you have a few different messages to decode / encode, it is doable to
implement everything in the single file ``${middleware}_mw.py``. However, if you
have a lot of different message to encode / decode, it is preferable to split
each processing in a separate file. For this purpose, you need to create a
directory ``${middleware}``. Each file will implement a special processing for
a given type of message. These files need to implement at least two different methods :

- ``init_extra_module(self, component_instance, function, mw_data)`` : this
  method will register the new serializer to its associated middleware
- a method to really serialize / deserialize the component input / output.

When you write a method which serializes the component output, you must not
modify the associated component. You are allowed to make computations to adapt
the internal MORSE data structure (``local_data``) to the needs of your
specific middleware (it is the main goal of this layer in fact).
If you feel that some sensor / actuator requires
more information, you must add (propose a patch, or inherit and extend) it at
the sensor level and not at the middleware level, so every potential user
can profit from it (and not only people using this specific middleware).
