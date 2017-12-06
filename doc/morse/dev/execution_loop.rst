MORSE execution loop
====================

The following figure depicts Morse's general behaviour:

.. image:: ../../media/simulation_main_loop_overview.svg
   :class: full_image
   :width: 600
   :align: center

After the initialisation phase described :doc:`here <entry_point>`, the
simulator starts a loop. At each loop execution, the simulator runs the code for each sensor, each
actuator (depending their frequencies), and handles services. It is
important to understand that, during any one loop execution, all the sensors and
actuators are called with the same graphical and physical contexts (robots positions,
sensor states, etc.).

.. warning::

	At the moment, the execution order between the different sensors and
	actuators of a scene in any one loop execution is not defined. So, do not rely
	on any particular execution order when implementing the behaviour of your component.

.. warning::

	If the behaviour of a component takes too much time, it is the whole
	simulation loop which is slown down (including the physics). Make sure your
	components are fast enough. It is possible to rewrite the logic in C if
	the Python version is too slow.


Behaviour of a sensor
---------------------

When Blender calls the method :py:meth:`morse.core.sensor.Sensor.action`, the
following things are done:

#. update the sensor's position
#. call the sensor's overridden ``default_action`` method
#. apply in order each of the ``output_modifiers`` functions (modify the sensor's content)
#. apply in order each of the ``output_functions`` functions (output the sensor's content to different clients)

Behaviour of an actuator
------------------------

When Blender calls the method :py:meth:`morse.core.actuator.Actuator.action`
the following things are done:

#. apply in order each of the ``input_functions`` functions (receive input from
   different clients) 
#. apply in order each of the ``input_modifiers`` functions (if needed)
#. call the actuator's overridden ``default_action`` method

.. warning::

	The mechanics allows for an actuator to have multiples clients, but in
	practice, the behaviour in such cases is not well defined. So, we recommend
	that you have only one client for one actuator.

Service handling
----------------

This part is explained in detail :doc:`here <services_internal>`.

