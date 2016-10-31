Morse Component object model
============================

The following diagram is a partial view of the class hierarchy currently used in
MORSE.

.. image:: ../../media/morse_uml.png
   :align: center 

The :py:class:`morse.core.abstractobject.AbstractObject` contains the logic for
handling services (:tag:`service`). It also contains an ordered dictionary
``local_data``, which is used to communicate between Morse's layers.
It is necessary to use an ``OrderedDict`` and not a standard ``dict``
because some middleware serialization depends on data being in a fixed
order (and it is is not possible with ``dict``).

The :py:class:`morse.core.object.Object` contains the logic for ``real
objects``. They own a reference to their associated Blender object
:py:data:`morse.core.object.Object.bge_object`, and contain the basic logic
about their position.

The :py:class:`morse.core.sensor.Sensor` and
:py:class:`morse.core.actuator.Actuator` classes are specializations of
the :py:class:`morse.core.object.Object` class. They add some arrays to store their
modifiers, implement datastream handlers, and override the base class' ``action``
method. See :doc:`this document <./execution_loop>` to learn what happens when
these method are called.


