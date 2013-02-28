Morse Component object model
============================

The following diagram shows the class hierarchy currently used in MORSE.

.. image:: ../../media/morse_uml.png
   :align: center 

The main entry point for Blender for each component is the method ``action``.
Yet, it is not supposed to be overridden by leaf-classes. To modify the
behaviour of a component, you need to modify the method ``default_action``. The
action of ``action`` depends on whether the component is a sensor or an actuator
(robots don't do anything by themselves). 

Component internal data
_______________________

The base :py:class:`morse.core.object.AbstractObject` defines an ordered
dictionary called ``local_data``. This is the place where all sensors and
actuators store the variables with the information that can eventually be
share through the middleware connections.  (for example, it will contain the
position for a GPS sensor, or the destination coordinates given to a motion
actuator) The order in which this variables are defined inside of a component
is important, since it will also be the default order in which the data is
exported via the middlewares (in automatic serialization).

Additionally, component classes can define any other variables internally, but
only the information in them will not be visible outside of MORSE.

