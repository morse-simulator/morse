Morse Component object model
============================

The following diagram a partial view of the class hierarchy currently used in
MORSE.

.. image:: ../../media/morse_uml.png
   :align: center 

The :py:class:`morse.core.abstractobject.AbstractObject` contains the logic to
handle services (:tag:`service`). It contains also an ordered dictionary
``local_data``, which is used to communicate between the different layer of
Morse. It is important to have an ``OrderedDict`` and not a standard ``Dict``
because order are used for some middleware serialization (and it is not well
defined in the case of ``Dict``).

The :py:class:`morse.core.object.Object` contains the logic for ``real
objects``. They own a reference to their associated blender object
:py:data:`morse.core.object.Object.bge_object`, and contain the basic logic
about their spatial relation. 

:py:class:`morse.core.sensor.Sensor` and
:py:class:`morse.core.actuator.Actuator` are specialization of this class,
respectively for sensor and actuator. They add some arrays to store their
modifiers, and their datastream handlers, and override their ``action``
method. See :doc:`this document <./execution_loop>` to learn what happens when
these method are called.


