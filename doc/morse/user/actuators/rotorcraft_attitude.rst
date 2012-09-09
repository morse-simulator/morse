Rotorcraft Attitude actuator
============================

This actuator reads roll,pitch, yaw rate and thrust commands as e.g. used
to manually control a quadrotor via RC or by higher level control loops.
This controller is meant to be used by quadrotors and similar flying robots
with ``Rigid Body`` physics in blender.
It is a simple PD-controller which applies torques to the robot
to change and control the attitude. The yaw-rate input is integrated to yield
an absolute yaw setpoint for the controller.
Thrust is directly applied as force in z-direction of the robot.

Files
-----

-  Blender: ``$MORSE_ROOT/data/actuators/rotorcraft_attitude.blend``
-  Python: ``$MORSE_ROOT/src/morse/actuators/rotorcraft_attitude.py``

Local data
----------

- **roll**: (float) desired roll angle in radians
- **pitch**: (float) desired pitch angle in radians
- **yaw**: (float) desired yaw rate in rad/s
- **thrust**: (float) thrust from 0 .. 1 (= 0 .. 100%)

.. note:: The angles are given in aerospace NED convention.

Configurable parameters
-----------------------

- **RollPitchPgain**: (float) proportional gain for roll/pitch control.
- **RollPitchDgain**: (float) derivative gain for roll/pitch control.
- **YawPgain**: (float) proportional gain for yaw control.
- **YawDgain**: (float) derivative gain for yaw control.
- **ThrustFactor**: (float) multiplication factor for applied thrust force in N.

Applicable modifiers
--------------------

- none
