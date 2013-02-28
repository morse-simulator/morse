MORSE execution loop
====================

The following figure depicts the general behaviour of Morse:

.. image:: ../../media/simulation_main_loop_overview.png
   :width: 600
   :align: center

After the initialisation phase described :doc:`here <entry_point>`, the
simulator goes in this big loop, including the execution of each sensor, each
actuator (depending their frequencies), and the handling of services. It is
important to understand that, during one loop execution, each sensor and
actuator is called with the same graphic and physic context (robots positions,
sensors position, ...).

.. warning::

	At the moment, the execution order between the different sensors and
	actuators of a scene is not defined. Do not rely on such order to
	guarantee the behaviour of your component.

.. warning::

	If the behaviour of one component takes too much time, it is the whole
	simulation loop which is slow down (including physics). Make sure your
	components are fast enough. It is possible to rewrite the logic in C if
	the python version is too slow.


Behaviour of a sensor
---------------------

When Blender calls the method :py:meth:`morse.core.sensor.Sensor.action`, the
following things happen:

#. update of the position of the sensor
#. call overridden ``default_action``
#. apply in order each function of ``output_modifiers`` (modify the content of
   the sensor)
#. apply in order each function of ``output_functions`` (output the content of
   the sensor to different clients)

Behaviour of an actuator
------------------------

When Blender calls the method :py:meth:`morse.core.actuator.Actuator.action`
the following things happen:

#. apply in order each function of ``input_functions`` (receive input from
   different clients) 
#. apply in order each function of ``input_modifiers`` (if needed)
#. call overridden ``default_action``

.. warning::

	The mechanic allows to have multiples client for on actuator, but in
	practice, the behaviour is not well defined, so it is better to make sure
	that you have only one client for one actuator.

Service handling
----------------

This part is explained with great details :doc:`here <services_internal>`.

