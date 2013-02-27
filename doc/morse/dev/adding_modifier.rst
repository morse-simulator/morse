Adding a new modifier
=====================

The concept of a modifier is relatively simple. Their only function is to
change the data stored in variables in the corresponding component, by using
the concept of :doc:`hooks <../user/hooks>`. 

A modifier is defined by a Python script.
The Python script is responsible for altering the data by changing directly the
information stored in the ``local_data`` dictionary of each component.

.. note:: 

    Be aware that once the data has been changed, there is no way to get the
    original information back.  If there is a need to have both the clean and
    the modified data for a particular sensor, the recommended method is to add
    two sensors of the same type, and only bind one of them with the modifier.

The python part 
---------------

There is no strict class hierarchy for modifiers, we rely on python duck
typing. We only expect than a modifier exposes a method ``register_component``,
similarly to middleware, with the following prototype : ::

  def register_component(self, component_name, component_instance, mod_data)

The method is responsible to add the right modifier specified in ``mod_data``
in the list of modifier of ``component_instance`` ( input_modifiers or
output_modifiers ). 

Modifier functions must have the prototype: ::

  def modifier_name(self, component_instance)

In your modifier function, you must only access to the array ``local_data``
of the component.

Examples
--------

`NED_modifier.py <http://trac.laas.fr/git/morse/tree/src/morse/modifiers/ned.py>`_ 
shows a simple example for a modifier.

`GPS_noise.py <http://trac.laas.fr/git/morse/tree/src/morse/modifiers/gps_noise.py>`_ 
calls a C routine to add Gaussian noise to a GPS output.

