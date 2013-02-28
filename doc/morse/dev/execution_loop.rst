MORSE execution loop
====================


.. image:: ../../media/simulation_main_loop_overview.png
   :width: 600
   :align: center

Behaviour of a sensor
_____________________

When Blender calls the method :py:meth:`morse.core.sensor.Sensor.action`, the
following things happen:

#. update of the position of the sensor
#. call overridden ``default_action``
#. apply in order each function of ``output_modifiers`` (modify the content of
   the sensor)
#. apply in order each function of ``output_functions`` (output the content of
   the sensor to different clients)

Behaviour of an actuator
________________________

When Blender calls the method :py:meth:`morse.core.actuator.Actuator.action`
the following things happen:

#. apply in order each function of ``input_functions`` (receive input from
  different clients)
#. apply in order each function of ``input_modifiers`` (if needed)
#. call overridden ``default_action``

