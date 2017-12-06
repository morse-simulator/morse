Morse Component object model
============================

The following diagram a partial view of the class hierarchy currently used in
MORSE.

.. image:: ../../media/morse_uml.png
   :align: center 

The :py:class:`morse.core.abstractobject.AbstractObject` contains the logic for
handling services (:tag:`service`). It also contains an ordered dictionary
``local_data``, which is used to communicate between Morse's different layers.
It is necessary to use an (insertion-ordered) ``OrderedDict`` and not a standard ``dict``
because some middleware serialization depends on having a fixed reproducible key order (and this
isn't possible with ``dict``).

The :py:class:`morse.core.object.Object` contains the logic for ``real
objects``. They own a reference to their associated blender object
:py:data:`morse.core.object.Object.bge_object`, and contain the basic logic
about their spatial relation. 

:py:class:`morse.core.sensor.Sensor` and
:py:class:`morse.core.actuator.Actuator` are specializations of the :py:class:`morse.core.object.Object`class,
for sensors and actuators. They add some arrays to store their
modifiers, and their datastream handlers, and override their base class's ``action``
method. See :doc:`this document <./execution_loop>` to learn what happens when
these method are called.


