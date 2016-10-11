Adding a new modifier
=====================

The concept of a modifier is relatively simple. Their only function is to
change the data stored in variables in the corresponding component, for
example to add noise to the information, or by applying some generic
geometric transformation.

A modifier is defined by a Python script.
The Python script is responsible for altering the data by directly changing the
information stored in each component's ``local_data`` dictionary.

.. note:: 

    Be aware that once the data has been changed, there is no way to get the
    original information back.  If there is a need to have both the clean and
    the modified data for a particular sensor, the recommended method is to add
    two sensors of the same type, and only bind one of them to the modifier.

Adding a new modifier
---------------------

Adding a new modifier is a matter of implementing a new class which
derives from :py:class:`morse.modifiers.abstract_modifier.AbstractModifier`.

The only mandatory method to specialize is the ``modify`` method which
contains the actual modification of component data. In this method,
you can use ``self.data`` to access to the ``local_data`` field of the
associated component.


It can also be useful to override the ``initialize`` and ``finalize``
methods, that contain the initialization code, and the
finalization code. Do not override ``__init__`` and ``__del__``.

If you want to add some parameters to your modifier and to get their
value during initialization, you can use the ``parameter`` method.

Let see an example with a custom modifier that switches ``x`` and ``y``
and the puts ``z`` to a constant value (set from a parameter).

.. code-block:: python

    from morse.modifiers.abstract_modifier import AbstractModifier

    class MyModifier(AbstractModifier):
        def initialize(self):
            """ initialization of parameters ... """
            self.z = self.pamateter("z", default=0)

        def modify(self):
            """" modify the data in this method """
            x = self.data['x']
            self.data['x'] = self.data['y']
            self.data['y'] = x
            self.data['z'] = self.z

Now, you can test your new modifier directly in the builder:

.. code-block:: python

    from morse.builder import *

    atrv = ATRV()

    pose = Pose()
    atrv.append(pose)
    pose.alter('', 'Path.to.MyModifier')

    env = Environment('empty', fastmode=True)

Lastly, if you want to use it more easily, you can add some entries in
:py:data:`morse.builder.data.MORSE_DATASTREAM_DICT`.

Examples
--------

:py:mod:`morse.modifiers.ned` shows a simple example for a modifier.

