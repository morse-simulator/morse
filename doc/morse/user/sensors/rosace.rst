Victim sensor (Rosace sensor)
=============================

This is a multi functional component specific for the ROSACE scenario,
where the robot must be able to aid human victims.
The sensor is capable of detecting any victim located within a cone in front of
the robot, with a range delimited in the properties of the Blender object.
The output of the sensor is a list of the robots and their positions in the
simulated world.
This sensor works only with the :doc:`human victim <../others/victim>` object.

Additionally, the sensor provides a number of services related to the
capabilities of the robot to help the nearest victim:

- Report on the condition of a victim
- Report the capabilities of the robot
- Heal a victim (if the robot has compatible capabilities with the requirements
  of the victim)

In the test scenarios, human victims are shown in *red*. When a robot approaches,
if it has the adequate capabilities, it will be able to help the victims.
When issued the command, the sensor will gradually change the colour of the
victim to *green*, and its status to healthy.
It will detect if a victim is in front of the robot. When instructed to heal the victim,
it will change the Game Properties of the object to reduce its ``injured`` value.

Files
-----

- Blender: ``$MORSE_ROOT/data/sensors/rosace.blend``
- Python: ``$MORSE_ROOT/src/morse/sensors/rosace.py``

Local data 
----------

- **victim_dict**: (dictionary) Contains an entry for each of the victims that
  is located inside the cone of the sensor. The value of each entry is a list
  containing three elements:

   - a list with the coordinates of the victim object
   - a list with the requirements of that victim
   - the severity (priority) value of the victim

Configurable parameters
-----------------------

- **Heal_range**: (float) maximum distance from which it is possible to heal a
  victim. Even if the victim can be detected by the sensor, it can't be healed
  unless its distance from the robot is less than this value.
- **Abilities**: (string) encoded string with a list of numbers, separated by
  comas, that represent the equipment capabilities of the robot. This information
  should be used by the operator of the robot to determine if it is capable of
  helping a victim or not.

The following parameters can be adjusted within the **Logic Bricks** of the
``Rosace_Sensor`` object in Blender, in the properties of the ``Radar`` sensor.

- **Freq**: (int) change the delay required to heal a victim. This number is
  expressed as the number of tics that are ignored before taking action. A lower
  number will produce a lower delay.

- **Angle**: (float) aperture angle of the radar capable of detecting the
  victims.
- **Distance**: (float) detection distance of the radar. Victims
  further away from the robot than this value will not be detected.


Services
--------

- **heal**: (Asynchronous service) Reduce the ``Severity`` value of the victim,
  and when the value reaches '0', change the ``Injured`` status of the victim
  to False.  When the victim is healed, the sensor sends out a reply,
  indicating the new status of the victim.

- **get_victim_severity**: (Synchronous service) This method will return the
  integer indicating the victim healing priority.

- **get_victim_requirements**: (Synchronous service) This method will return
  the string describing the type of injury sustained by the victim.

- **get_robot_abilities**: (Synchronous service) This method will return the
  string describing the abilities with which the robot is equipped, and which
  must match the requirements of the victim for the robot to be able to heal it.


Applicable modifiers
--------------------

No available modifiers
